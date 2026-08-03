#include "RouteDesign.h"

#include "Device.h"
#include "Module.h"
#include "TimingPath.h"
#include "Wire.h"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

struct Failure : std::runtime_error {
  using std::runtime_error::runtime_error;
};

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw Failure(message);
  }
}

NodeMask bit(int node) { return NodeMask{0, 1} << node; }

std::string randomToken(std::mt19937 &rng, const std::string &prefix) {
  static constexpr char alphabet[] = "abcdefghijklmnopqrstuvwxyz";
  std::uniform_int_distribution<int> pick(0, 25);
  std::string result = prefix;
  for (int i = 0; i < 12; ++i) {
    result.push_back(alphabet[pick(rng)]);
  }
  return result;
}

void resetGrid(int width, int height) {
  fpga::Device &device = fpga::Device::current();
  device.tile_grid.clear();
  device.grid_spec.size = {width, height};
  device.size_width = width;
  device.size_height = height;
  device.tile_grid.resize(static_cast<size_t>(width * height));
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      device.tile_grid[static_cast<size_t>(y * width + x)].coord = {x, y};
    }
  }
}

fpga::Wire tilePin(fpga::Coord coord, int local, const std::string &node) {
  fpga::Wire wire;
  wire.type = fpga::Wire::WIRE_TILE_PIN;
  wire.from = coord;
  wire.to = coord;
  wire.local = local;
  wire.src_wire_name = node;
  return wire;
}

fpga::Wire crossbar(fpga::Coord from, fpga::Coord to, int local, int src,
                    int dst, const std::string &from_name,
                    const std::string &src_name, const std::string &dst_name,
                    bool source) {
  fpga::Wire wire;
  wire.type = fpga::Wire::WIRE_CROSSBAR;
  wire.from = from;
  wire.to = to;
  wire.local = local;
  wire.jump = src;
  wire.dst = dst;
  wire.pos = source ? 0 : 1;
  wire.from_wire_name = from_name;
  wire.src_wire_name = src_name;
  wire.dst_wire_name = dst_name;
  return wire;
}

void leaseOwnedRoute(const std::vector<fpga::Wire> &route) {
  for (size_t index = 0; index < route.size(); ++index) {
    const fpga::Wire &wire = route[index];
    if (wire.shared) {
      continue;
    }
    fpga::Tile *tile =
        fpga::Device::current().getTile(wire.from.x, wire.from.y);
    require(tile != nullptr, "route references a tile outside the test grid");
    if (wire.type == fpga::Wire::WIRE_TILE_PIN) {
      if (index + 1 == route.size()) {
        tile->pin_state.leased_nodes |= bit(wire.local);
        tile->cb.local.local |= bit(wire.local);
      }
      continue;
    }
    tile->cb.src.jump |= bit(wire.jump);
    if (wire.pos == 0) {
      tile->cb.local.local |= bit(wire.local);
    } else if (wire.owns_dst) {
      tile->cb.dst.jump |= bit(wire.local);
    }
  }
}

struct TestDesign {
  rtl::Design design;
  std::vector<std::unique_ptr<rtl::Inst>> instances;

  rtl::Module &addModule() { return design.modules.emplace_back(); }

  rtl::Inst *addInst() {
    instances.push_back(std::make_unique<rtl::Inst>());
    return instances.back().get();
  }

  rtl::Net &addNet(rtl::Module &module, const std::string &name) {
    Referable<rtl::Net> &net = module.nets.emplace_back();
    net.name = name;
    return net;
  }

  void addRoute(rtl::Net &net, rtl::Inst *driver,
                const std::string &source_port, std::vector<fpga::Wire> route,
                const std::string &route_name) {
    rtl::Inst *sink = addInst();
    rtl::Inst *owner = addInst();
    owner->wires.push_back(std::move(route));
    size_t route_index = owner->wires.size() - 1;
    net.routes.push_back(rtl::NetRouteBinding{
        owner, route_index, driver, sink, source_port, "input", route_name});
    leaseOwnedRoute(owner->wires[route_index]);
    fpga::registerNetRouteTiles(net, owner->wires[route_index]);
  }
};

struct BuiltTree {
  rtl::Net *net = nullptr;
  rtl::Inst *driver = nullptr;
  std::string source_port;
  std::vector<std::string> route_names;
  size_t prefix_crossbars = 0;
};

BuiltTree addSharedTree(TestDesign &fixture, rtl::Module &module,
                        std::mt19937 &rng, int base_x, int prefix_crossbars,
                        int branch_count, const std::string &label) {
  BuiltTree tree;
  tree.net = &fixture.addNet(module, randomToken(rng, label + "_net_"));
  tree.driver = fixture.addInst();
  tree.source_port = randomToken(rng, label + "_port_");
  tree.prefix_crossbars = static_cast<size_t>(prefix_crossbars);

  int local = 10 + base_x;
  std::string current_node = randomToken(rng, label + "_origin_");
  std::vector<fpga::Wire> prefix;
  prefix.push_back(tilePin({base_x, 1}, local, current_node));
  for (int step = 0; step < prefix_crossbars; ++step) {
    int src = 40 + step;
    int dst = 100 + step;
    std::string src_name = randomToken(rng, label + "_src_");
    std::string dst_name = randomToken(rng, label + "_dst_");
    prefix.push_back(crossbar({base_x + step, 1}, {base_x + step + 1, 1},
                              step == 0 ? local : 99 + step, src, dst,
                              current_node, src_name, dst_name, step == 0));
    current_node = dst_name;
  }

  std::vector<fpga::Wire> trunk = prefix;
  trunk.push_back(tilePin({base_x + prefix_crossbars, 1}, 200,
                          randomToken(rng, label + "_trunk_pin_")));
  tree.route_names.push_back(randomToken(rng, label + "_trunk_"));
  fixture.addRoute(*tree.net, tree.driver, tree.source_port, std::move(trunk),
                   tree.route_names.back());

  for (int branch_index = 0; branch_index < branch_count; ++branch_index) {
    std::vector<fpga::Wire> branch = prefix;
    for (fpga::Wire &wire : branch) {
      wire.shared = true;
      wire.owns_dst = false;
    }
    int branch_src = 300 + branch_index;
    int branch_dst = 400 + branch_index;
    fpga::Coord branch_from{base_x + prefix_crossbars, 1};
    fpga::Coord branch_to{base_x + prefix_crossbars + 1, 2 + branch_index};
    branch.push_back(crossbar(branch_from, branch_to, 99 + prefix_crossbars,
                              branch_src, branch_dst, current_node,
                              randomToken(rng, label + "_branch_src_"),
                              randomToken(rng, label + "_branch_dst_"), false));
    branch.push_back(tilePin(branch_to, 500 + branch_index,
                             randomToken(rng, label + "_branch_pin_")));
    tree.route_names.push_back(randomToken(rng, label + "_branch_"));
    fixture.addRoute(*tree.net, tree.driver, tree.source_port,
                     std::move(branch), tree.route_names.back());
  }
  return tree;
}

std::vector<fpga::Wire> &boundRoute(rtl::NetRouteBinding &binding) {
  require(binding.owner != nullptr &&
              binding.route_index < binding.owner->wires.size(),
          "invalid test route binding");
  return binding.owner->wires[binding.route_index];
}

bool regionHasLeases(int first_x, int last_x, int first_y, int last_y) {
  for (int y = first_y; y <= last_y; ++y) {
    for (int x = first_x; x <= last_x; ++x) {
      const fpga::Tile *tile = fpga::Device::current().getTile(x, y);
      if (tile &&
          (tile->cb.src.jump != NodeMask{} || tile->cb.dst.jump != NodeMask{} ||
           tile->cb.local.local != NodeMask{} ||
           tile->pin_state.leased_nodes != NodeMask{})) {
        return true;
      }
    }
  }
  return false;
}

void releasing_one_conflicting_owner_preserves_the_survivor() {
  resetGrid(4, 4);
  TestDesign fixture;
  rtl::Module &module = fixture.addModule();
  module.nets.reserve(2);

  rtl::Net &first = fixture.addNet(module, "first");
  rtl::Net &second = fixture.addNet(module, "second");
  rtl::Inst *first_driver = fixture.addInst();
  rtl::Inst *second_driver = first_driver;
  std::vector<fpga::Wire> first_route{
      tilePin({1, 1}, 4, "first_local"),
      crossbar({1, 1}, {2, 1}, 4, 17, 23, "first_local", "shared_src",
               "first_dst", true),
      tilePin({2, 1}, 8, "first_pin"),
  };
  std::vector<fpga::Wire> second_route{
      tilePin({1, 1}, 4, "first_local"),
      crossbar({1, 1}, {2, 1}, 4, 17, 23, "first_local", "shared_src",
               "first_dst", true),
      crossbar({2, 1}, {2, 2}, 23, 18, 24, "first_dst", "branch_src",
               "second_dst", true),
      tilePin({2, 2}, 9, "second_pin"),
  };
  second_route[0].shared = true;
  second_route[1].shared = true;
  second_route[1].owns_dst = false;
  second_route[1].owns_landing = false;
  fixture.addRoute(first, first_driver, "first_output", std::move(first_route),
                   "first_route");
  fixture.addRoute(second, second_driver, "first_output",
                   std::move(second_route), "second_route");

  fpga::Tile *shared_tile = fpga::Device::current().getTile(1, 1);
  require(shared_tile != nullptr &&
              (shared_tile->cb.src.jump & bit(17)) != NodeMask{},
          "test setup did not lease the conflicting source node");

  // Removing the prefix owner must transfer ownership to its shared survivor.
  require(fpga::unrouteNetRoute(first, 0),
          "failed to remove the first conflicting route");
  require(
      (shared_tile->cb.src.jump & bit(17)) != NodeMask{},
      "removing one owner cleared a source lease still owned by another route");

  // Once the final owner is removed, the same physical lease must become free.
  require(fpga::unrouteNetRoute(second, 0),
          "failed to remove the surviving conflicting route");
  require((shared_tile->cb.src.jump & bit(17)) == NodeMask{},
          "removing the final owner left a stale source lease");
}

void non_owning_shared_prefix_does_not_leak_removed_owner_lease() {
  resetGrid(5, 4);
  TestDesign fixture;
  rtl::Module &module = fixture.addModule();
  module.nets.reserve(2);
  rtl::Net &owner_net = fixture.addNet(module, "owner");
  rtl::Net &replica_net = fixture.addNet(module, "replica");
  rtl::Inst *driver = fixture.addInst();

  std::vector<fpga::Wire> owner_route{
      tilePin({1, 1}, 4, "origin"),
      crossbar({1, 1}, {2, 1}, 4, 17, 23, "origin", "prefix_src", "prefix_dst",
               true),
      tilePin({2, 1}, 8, "owner_pin"),
  };
  std::vector<fpga::Wire> replica_route = owner_route;
  for (fpga::Wire &fragment : replica_route) {
    fragment.shared = true;
    fragment.owns_dst = false;
  }
  replica_route.back() = tilePin({2, 1}, 9, "replica_pin");

  fixture.addRoute(owner_net, driver, "output", std::move(owner_route),
                   "owner_route");
  rtl::Inst *unrelated_driver = fixture.addInst();
  fixture.addRoute(replica_net, unrelated_driver, "other_output",
                   std::move(replica_route), "replica_route");

  fpga::Tile *prefix_tile = fpga::Device::current().getTile(1, 1);
  require(prefix_tile && (prefix_tile->cb.src.jump & bit(17)) != NodeMask{},
          "shared-prefix leak test did not lease its owning source");

  // A shared route fragment references the tree but does not own its lease.
  require(fpga::unrouteNetRoute(owner_net, 0),
          "shared-prefix leak test could not remove the physical owner");
  require((prefix_tile->cb.src.jump & bit(17)) == NodeMask{},
          "non-owning shared-prefix replica leaked a removed source lease");
}

void cross_source_shared_prefix_requires_same_source_owner() {
  resetGrid(5, 4);
  TestDesign fixture;
  rtl::Module &module = fixture.addModule();
  module.nets.reserve(2);
  rtl::Net &owner_net = fixture.addNet(module, "owner_source");
  rtl::Net &stale_net = fixture.addNet(module, "stale_source");
  rtl::Inst *owner_driver = fixture.addInst();
  rtl::Inst *stale_driver = fixture.addInst();

  std::vector<fpga::Wire> owner_route{
      tilePin({1, 1}, 4, "owner_local"),
      crossbar({1, 1}, {2, 1}, 4, 17, 23, "owner_local", "shared_src",
               "shared_dst", true),
      tilePin({2, 1}, 8, "owner_pin"),
  };
  std::vector<fpga::Wire> stale_route{
      tilePin({0, 2}, 10, "stale_local"),
      crossbar({0, 2}, {1, 2}, 10, 18, 24, "stale_local", "stale_src",
               "stale_dst", true),
      crossbar({1, 1}, {2, 1}, 4, 17, 23, "owner_local", "shared_src",
               "shared_dst", true),
      tilePin({2, 1}, 9, "stale_pin"),
  };
  // Reproduce the production bug: a foreign shared span appears after a
  // private route prefix, so checking only route.front().shared misses it.
  stale_route[2].shared = true;
  stale_route[2].owns_dst = false;

  fixture.addRoute(owner_net, owner_driver, "owner_output",
                   std::move(owner_route), "owner_route");
  fixture.addRoute(stale_net, stale_driver, "stale_output",
                   std::move(stale_route), "stale_route");

  std::vector<pnr::RouteDesign::RouteTask> tasks;
  require(pnr::RouteDesign::repairStaleSharedRoutePrefixes(fixture.design,
                                                           tasks) == 1,
          "cross-source shared prefix was accepted without a same-source owner");
  require(tasks.size() == 1 && tasks.front().net == &stale_net,
          "shared-prefix repair did not requeue only the stale source tree");
  require(!boundRoute(owner_net.routes.front()).empty(),
          "shared-prefix repair removed the valid physical owner");
  require(stale_net.routes.front().owner == nullptr,
          "shared-prefix repair retained the cross-source replica");
  fpga::Tile *tile = fpga::Device::current().getTile(1, 1);
  require(tile && (tile->cb.src.jump & bit(17)) != NodeMask{},
          "shared-prefix repair released the valid owner's lease");
}

void packed_connection_transfers_prefix_ownership_across_rtl_nets() {
  resetGrid(6, 4);
  TestDesign fixture;
  rtl::Module &module = fixture.addModule();
  module.nets.reserve(2);
  rtl::Net &packed_net = fixture.addNet(module, "packed_connection");
  rtl::Net &fanout_net = fixture.addNet(module, "external_fanout");
  rtl::Inst *driver = fixture.addInst();

  std::vector<fpga::Wire> owned_prefix{
      tilePin({1, 1}, 4, "source_local"),
      crossbar({1, 1}, {2, 1}, 4, 17, 23, "source_local", "prefix_src",
               "prefix_dst", true),
  };
  std::vector<fpga::Wire> packed_route = owned_prefix;
  packed_route.push_back(crossbar({2, 1}, {3, 1}, 23, 19, 29, "prefix_dst",
                                  "packed_src", "packed_dst", false));
  packed_route.push_back(tilePin({3, 1}, 8, "packed_pin"));

  std::vector<fpga::Wire> fanout_route = owned_prefix;
  for (fpga::Wire &fragment : fanout_route) {
    fragment.shared = true;
    fragment.owns_dst = false;
  }
  fanout_route.push_back(crossbar({2, 1}, {3, 2}, 23, 21, 31, "prefix_dst",
                                  "fanout_src", "fanout_dst", false));
  fanout_route.push_back(tilePin({3, 2}, 9, "fanout_pin"));

  fixture.addRoute(packed_net, driver, "output", std::move(packed_route),
                   "packed_route");
  fixture.addRoute(fanout_net, driver, "output", std::move(fanout_route),
                   "fanout_route");
  rtl::NetRouteBinding &packed_binding = packed_net.routes.front();
  rtl::NetRouteBinding &fanout_binding = fanout_net.routes.front();

  require(fpga::unrouteNetConnection(
              packed_net, packed_binding.from, packed_binding.to,
              packed_binding.from_port, packed_binding.to_port) == 1,
          "packed cross-net connection was not removed");
  std::vector<fpga::Wire> &surviving_route = boundRoute(fanout_binding);
  require(!surviving_route[0].shared && !surviving_route[1].shared,
          "cross-net fanout did not inherit its source-prefix ownership");
  fpga::Tile *prefix_tile = fpga::Device::current().getTile(1, 1);
  require(prefix_tile && (prefix_tile->cb.src.jump & bit(17)) != NodeMask{},
          "cross-net ownership transfer released the surviving source prefix");
}

void moving_parent_transfers_private_suffix_to_descendant() {
  resetGrid(8, 5);
  TestDesign fixture;
  rtl::Module &module = fixture.addModule();
  module.nets.reserve(3);
  rtl::Net &trunk_net = fixture.addNet(module, "source_trunk");
  rtl::Net &parent_net = fixture.addNet(module, "parent_fanout");
  rtl::Net &descendant_net = fixture.addNet(module, "descendant_fanout");
  rtl::Inst *driver = fixture.addInst();

  std::vector<fpga::Wire> trunk_prefix{
      tilePin({1, 1}, 4, "source_local"),
      crossbar({1, 1}, {2, 1}, 4, 17, 23, "source_local", "trunk_src",
               "trunk_dst", true),
  };
  std::vector<fpga::Wire> trunk_route = trunk_prefix;
  trunk_route.push_back(tilePin({2, 1}, 7, "trunk_pin"));
  fixture.addRoute(trunk_net, driver, "output", std::move(trunk_route),
                   "trunk_route");

  std::vector<fpga::Wire> parent_route = trunk_prefix;
  for (fpga::Wire &fragment : parent_route) {
    fragment.shared = true;
    fragment.owns_dst = false;
  }
  parent_route.insert(parent_route.end(),
                      {
                          crossbar({2, 1}, {3, 1}, 23, 18, 24, "trunk_dst",
                                   "parent_src_0", "parent_dst_0", false),
                          crossbar({3, 1}, {4, 1}, 24, 19, 25, "parent_dst_0",
                                   "parent_src_1", "parent_dst_1", false),
                          tilePin({4, 1}, 8, "parent_pin"),
                      });

  // This route branches from the parent's private suffix, so every copied
  // fragment is shared until its own physical branch begins at tile (4,1).
  std::vector<fpga::Wire> descendant_route(parent_route.begin(),
                                           parent_route.end() - 1);
  for (fpga::Wire &fragment : descendant_route) {
    fragment.shared = true;
    fragment.owns_dst = false;
  }
  descendant_route.push_back(crossbar({4, 1}, {5, 2}, 25, 20, 26,
                                      "parent_dst_1", "descendant_src",
                                      "descendant_dst", false));
  descendant_route.push_back(tilePin({5, 2}, 9, "descendant_pin"));

  fixture.addRoute(parent_net, driver, "output", std::move(parent_route),
                   "parent_route");
  fixture.addRoute(descendant_net, driver, "output",
                   std::move(descendant_route), "descendant_route");

  // Moving the parent sink must retain its physical prefix as shared state
  // and transfer ownership of every copied node to the descendant branch.
  require(fpga::invalidateMovedSinkRoute(parent_net, 0),
          "moving the parent sink did not invalidate its destination suffix");
  std::vector<fpga::Wire> &parent = boundRoute(parent_net.routes.front());
  std::vector<fpga::Wire> &descendant =
      boundRoute(descendant_net.routes.front());
  require(parent.size() == 4, "moving the parent did not retain exactly the "
                              "descendant's shared prefix");
  for (const fpga::Wire &fragment : parent) {
    require(fragment.shared,
            "moved parent retained private ownership of a descendant prefix");
  }
  for (size_t index = 0; index < trunk_prefix.size(); ++index) {
    require(descendant[index].shared,
            "ownership transfer stole an already-shared prefix from its trunk");
  }
  for (size_t index = trunk_prefix.size(); index < parent.size(); ++index) {
    require(!descendant[index].shared, "descendant did not inherit ownership "
                                       "of the moved parent's private suffix");
  }
  for (int source : {17, 18, 19}) {
    fpga::Tile *tile = fpga::Device::current().getTile(source - 16, 1);
    require(tile && (tile->cb.src.jump & bit(source)) != NodeMask{},
            "moving the parent released a physical prefix still used by its "
            "descendant");
  }

  // Once the descendant is removed, the moved parent's retained partial prefix
  // remains a live route user and must keep its continuation resources leased.
  require(fpga::unrouteNetRoute(descendant_net, 0),
          "failed to remove the descendant route owner");
  fpga::Tile *trunk_tile = fpga::Device::current().getTile(1, 1);
  require(trunk_tile && (trunk_tile->cb.src.jump & bit(17)) != NodeMask{},
          "removing the descendant released its independent trunk owner");
  for (int source : {18, 19}) {
    fpga::Tile *tile = fpga::Device::current().getTile(source - 16, 1);
    require(
        tile && (tile->cb.src.jump & bit(source)) != NodeMask{},
        "removing the descendant released the moved parent's retained prefix");
  }
  require(fpga::unrouteNetRoute(parent_net, 0),
          "failed to remove the moved parent's retained prefix");
  for (int source : {18, 19}) {
    fpga::Tile *tile = fpga::Device::current().getTile(source - 16, 1);
    require(
        tile && (tile->cb.src.jump & bit(source)) == NodeMask{},
        "removing the final retained-prefix user left its source lease set");
  }
  require(fpga::unrouteNetRoute(trunk_net, 0),
          "failed to remove the independent trunk route");
  require((trunk_tile->cb.src.jump & bit(17)) == NodeMask{},
          "removing the final trunk owner left its source lease set");
}

void moving_parent_distributes_ownership_across_branch_depths() {
  resetGrid(9, 6);
  TestDesign fixture;
  rtl::Module &module = fixture.addModule();
  module.nets.reserve(3);
  rtl::Net &parent_net = fixture.addNet(module, "parent");
  rtl::Net &shallow_net = fixture.addNet(module, "shallow_descendant");
  rtl::Net &deep_net = fixture.addNet(module, "deep_descendant");
  rtl::Inst *driver = fixture.addInst();

  std::vector<fpga::Wire> parent_route{
      tilePin({1, 1}, 4, "origin"),
      crossbar({1, 1}, {2, 1}, 4, 17, 23, "origin", "owned_0", "land_0", true),
      crossbar({2, 1}, {3, 1}, 23, 18, 24, "land_0", "owned_1", "land_1",
               false),
      crossbar({3, 1}, {4, 1}, 24, 19, 25, "land_1", "owned_2", "land_2",
               false),
      tilePin({4, 1}, 8, "parent_pin"),
  };

  // Register the shallow descendant first so it can inherit only the first
  // part of the parent's private path.
  std::vector<fpga::Wire> shallow(parent_route.begin(),
                                  parent_route.begin() + 3);
  for (fpga::Wire &fragment : shallow) {
    fragment.shared = true;
    fragment.owns_dst = false;
  }
  shallow.push_back(crossbar({3, 1}, {4, 2}, 24, 20, 26, "land_1",
                             "shallow_src", "shallow_dst", false));
  shallow.push_back(tilePin({4, 2}, 9, "shallow_pin"));

  // The deeper descendant shares the complete private path and must inherit
  // the remaining fragment that the shallow branch cannot represent.
  std::vector<fpga::Wire> deep(parent_route.begin(), parent_route.end() - 1);
  for (fpga::Wire &fragment : deep) {
    fragment.shared = true;
    fragment.owns_dst = false;
  }
  deep.push_back(crossbar({4, 1}, {5, 3}, 25, 21, 27, "land_2", "deep_src",
                          "deep_dst", false));
  deep.push_back(tilePin({5, 3}, 10, "deep_pin"));

  fixture.addRoute(parent_net, driver, "output", std::move(parent_route),
                   "parent_route");
  fixture.addRoute(shallow_net, driver, "output", std::move(shallow),
                   "shallow_route");
  fixture.addRoute(deep_net, driver, "output", std::move(deep), "deep_route");

  require(fpga::invalidateMovedSinkRoute(parent_net, 0),
          "moving the parent did not invalidate its split-depth destination");
  for (int source : {17, 18, 19}) {
    fpga::Tile *tile = fpga::Device::current().getTile(source - 16, 1);
    require(tile && (tile->cb.src.jump & bit(source)) != NodeMask{},
            "split-depth transfer released a surviving physical prefix");
    require(
        fpga::findNetOwnersByNode(*tile, fpga::CB_NODE_SRC, source).size() == 1,
        "split-depth transfer did not leave exactly one owner per source node");
  }
}

void incomplete_owner_keeps_completed_shared_sibling_valid() {
  resetGrid(7, 4);
  TestDesign fixture;
  rtl::Module &module = fixture.addModule();
  module.nets.reserve(2);
  rtl::Net &owner_net = fixture.addNet(module, "incomplete_owner");
  rtl::Net &sibling_net = fixture.addNet(module, "complete_sibling");
  rtl::Inst *driver = fixture.addInst();

  std::vector<fpga::Wire> owner_route{
      tilePin({1, 1}, 4, "source_local"),
      crossbar({1, 1}, {2, 1}, 4, 17, 23, "source_local", "owned_source",
               "owned_landing", true),
  };
  std::vector<fpga::Wire> sibling_route = owner_route;
  for (fpga::Wire &fragment : sibling_route) {
    fragment.shared = true;
    fragment.owns_dst = false;
  }
  sibling_route.push_back(crossbar({2, 1}, {3, 2}, 23, 18, 24, "owned_landing",
                                   "branch_source", "branch_landing", false));
  sibling_route.push_back(tilePin({3, 2}, 8, "sink_local"));

  fixture.addRoute(owner_net, driver, "output", std::move(owner_route),
                   "owner_route");
  fixture.addRoute(sibling_net, driver, "output", std::move(sibling_route),
                   "sibling_route");

  // Moving may leave the private trunk owner incomplete while a completed
  // sibling uses its prefix; this is a valid intermediate ownership state.
  std::vector<pnr::RouteDesign::RouteTask> tasks;
  require(pnr::RouteDesign::repairStaleSharedRoutePrefixes(fixture.design,
                                                           tasks) == 0,
          "incomplete trunk owner was omitted from shared-prefix validation");
  require(
      tasks.empty(),
      "valid completed sibling was destructively scheduled for prefix repair");
  require(!boundRoute(owner_net.routes.front()).empty() &&
              !boundRoute(sibling_net.routes.front()).empty(),
          "shared-prefix validation removed a valid incomplete-owner tree");
}

void randomized_shared_prefix_repairs_return_to_source_state() {
  std::mt19937 rng(0x5a17c0deU);
  std::uniform_int_distribution<int> prefix_length(2, 6);
  std::uniform_int_distribution<int> branch_count(2, 5);

  for (int test_index = 0; test_index < 10; ++test_index) {
    resetGrid(32, 12);
    TestDesign fixture;
    rtl::Module &module = fixture.addModule();
    module.nets.reserve(2);
    int damaged_length = prefix_length(rng);
    int damaged_branches = branch_count(rng);
    BuiltTree damaged = addSharedTree(fixture, module, rng, 1, damaged_length,
                                      damaged_branches, "damaged");
    BuiltTree control =
        addSharedTree(fixture, module, rng, 18, prefix_length(rng),
                      branch_count(rng), "control");

    if (test_index == 0) {
      // Reproduce a stale ownership alias: two endpoint bindings point at
      // one physical route slot before the source tree is repaired.
      require(fpga::unrouteNetRoute(*damaged.net, 1),
              "failed to release branch before constructing route-slot alias");
      damaged.net->routes[1].owner = damaged.net->routes[0].owner;
      damaged.net->routes[1].route_index = damaged.net->routes[0].route_index;
    }

    std::vector<pnr::RouteDesign::RouteTask> tasks;
    require(pnr::RouteDesign::repairStaleSharedRoutePrefixes(fixture.design,
                                                             tasks) == 0,
            "valid shared route tree was incorrectly classified as stale");
    require(tasks.empty(), "valid shared route tree generated repair tasks");

    std::uniform_int_distribution<int> corrupt_step(0, damaged_length - 1);
    size_t corrupt_index = 1 + static_cast<size_t>(corrupt_step(rng));
    std::vector<fpga::Wire> &trunk = boundRoute(damaged.net->routes.front());
    require(corrupt_index < trunk.size() - 1 &&
                trunk[corrupt_index].type == fpga::Wire::WIRE_CROSSBAR,
            "random corruption did not select an owned trunk fragment");
    trunk[corrupt_index].shared = true;

    // Shared is a route-tree annotation, not a reference-count exemption.
    // Every surviving route still references the prefix, so metadata damage
    // must not trigger destructive rerouting or release physical leases.
    size_t repaired =
        pnr::RouteDesign::repairStaleSharedRoutePrefixes(fixture.design, tasks);
    require(repaired == 0 && tasks.empty(),
            "collectively referenced prefix was destructively repaired");
    for (rtl::NetRouteBinding &binding : damaged.net->routes) {
      require(!boundRoute(binding).empty(),
              "collective prefix validation removed a surviving route");
    }
    require(regionHasLeases(1, 10, 0, 8),
            "collectively referenced prefix lost its physical leases");

    for (rtl::NetRouteBinding &binding : control.net->routes) {
      require(!boundRoute(binding).empty(),
              "repair modified an unrelated valid shared route tree");
    }
    require(regionHasLeases(18, 28, 0, 8),
            "repair released leases owned by an unrelated source tree");
  }
}

void cross_source_private_node_conflict_repairs_both_trees() {
  resetGrid(24, 8);
  TestDesign fixture;
  rtl::Module &module = fixture.addModule();
  module.nets.reserve(3);
  std::mt19937 rng(0x93cba5e1U);

  BuiltTree first = addSharedTree(fixture, module, rng, 1, 2, 1, "first");
  BuiltTree second = addSharedTree(fixture, module, rng, 9, 2, 1, "second");
  BuiltTree control = addSharedTree(fixture, module, rng, 17, 2, 1, "control");

  std::vector<pnr::RouteDesign::RouteTask> tasks;
  require(pnr::RouteDesign::repairStaleSharedRoutePrefixes(fixture.design,
                                                           tasks) == 0,
          "valid independent route trees were classified as conflicting");

  // Reproduce a final-state ownership bug: a private fragment in the second
  // driver tree names the same physical route node as the first driver tree.
  std::vector<fpga::Wire> &first_route = boundRoute(first.net->routes.front());
  std::vector<fpga::Wire> &second_route =
      boundRoute(second.net->routes.front());
  require(first_route.size() >= 3 && second_route.size() >= 3,
          "cross-source regression routes do not contain private crossbar "
          "fragments");
  fpga::Wire &first_fragment = first_route[1];
  fpga::Wire &second_fragment = second_route[1];
  second_fragment.from = first_fragment.from;
  second_fragment.to = first_fragment.to;
  second_fragment.local = first_fragment.local;
  second_fragment.jump = first_fragment.jump;
  second_fragment.dst = first_fragment.dst;
  second_fragment.from_wire_name = first_fragment.from_wire_name;
  second_fragment.src_wire_name = first_fragment.src_wire_name;
  second_fragment.dst_wire_name = first_fragment.dst_wire_name;
  const fpga::Coord conflicting_coord = first_fragment.from;
  const int conflicting_src = first_fragment.jump;

  const size_t first_routes = first.net->routes.size();
  const size_t second_routes = second.net->routes.size();
  size_t repaired =
      pnr::RouteDesign::repairStaleSharedRoutePrefixes(fixture.design, tasks);
  require(repaired == first_routes + second_routes,
          "cross-source repair did not remove both conflicting source trees");
  require(tasks.size() == first_routes + second_routes,
          "cross-source repair did not recreate every conflicting route task");

  std::vector<pnr::RouteDesign::RouteTask> generic_tasks;
  std::vector<pnr::RouteDesign::RouteTask> fanout_tasks;
  require(pnr::RouteDesign::scheduleSharedPrefixRepairs(tasks, generic_tasks,
                                                        fanout_tasks) == 2,
          "cross-source repair did not schedule one Generic seed per driver");
  require(generic_tasks.size() == 2,
          "cross-source repair lost a conflicting driver seed");
  require(fanout_tasks.size() == first_routes + second_routes - 2,
          "cross-source repair did not defer all dependent branches");

  for (rtl::NetRouteBinding &binding : first.net->routes) {
    require(binding.owner == nullptr,
            "first conflicting source retained old route storage");
  }
  for (rtl::NetRouteBinding &binding : second.net->routes) {
    require(binding.owner == nullptr,
            "second conflicting source retained old route storage");
  }
  for (rtl::NetRouteBinding &binding : control.net->routes) {
    require(!boundRoute(binding).empty(),
            "cross-source repair modified an unrelated control tree");
  }
  const fpga::Tile *conflicting_tile =
      fpga::Device::current().getTile(conflicting_coord.x, conflicting_coord.y);
  require(
      conflicting_tile &&
          (conflicting_tile->cb.src.jump & bit(conflicting_src)) == NodeMask{},
      "cross-source repair retained the deliberately duplicated source lease");
  require(regionHasLeases(17, 23, 0, 6),
          "cross-source repair released the unrelated control tree");
}

void protected_owner_wins_a_cross_source_node_conflict() {
  resetGrid(16, 8);
  TestDesign fixture;
  rtl::Module &module = fixture.addModule();
  module.nets.reserve(2);
  std::mt19937 rng(0x4f91a2c7U);

  BuiltTree reserved =
      addSharedTree(fixture, module, rng, 1, 2, 0, "reserved");
  BuiltTree ordinary =
      addSharedTree(fixture, module, rng, 9, 2, 0, "ordinary");
  reserved.net->route_protected = true;

  std::vector<fpga::Wire> &reserved_route =
      boundRoute(reserved.net->routes.front());
  std::vector<fpga::Wire> &ordinary_route =
      boundRoute(ordinary.net->routes.front());
  require(reserved_route.size() >= 3 && ordinary_route.size() >= 3,
          "protected-owner regression routes lack crossbar fragments");
  fpga::Wire &reserved_fragment = reserved_route[1];
  fpga::Wire &ordinary_fragment = ordinary_route[1];
  ordinary_fragment.from = reserved_fragment.from;
  ordinary_fragment.to = reserved_fragment.to;
  ordinary_fragment.local = reserved_fragment.local;
  ordinary_fragment.jump = reserved_fragment.jump;
  ordinary_fragment.dst = reserved_fragment.dst;
  ordinary_fragment.from_wire_name = reserved_fragment.from_wire_name;
  ordinary_fragment.src_wire_name = reserved_fragment.src_wire_name;
  ordinary_fragment.dst_wire_name = reserved_fragment.dst_wire_name;

  std::vector<pnr::RouteDesign::RouteTask> tasks;
  size_t repaired =
      pnr::RouteDesign::repairStaleSharedRoutePrefixes(fixture.design, tasks);

  // A protected infrastructure tree remains the physical owner while the
  // conflicting ordinary tree is removed and returned to Generic routing.
  require(repaired == ordinary.net->routes.size() && tasks.size() == 1
              && tasks.front().net == ordinary.net,
          "cross-source repair did not reschedule only the ordinary owner");
  require(!boundRoute(reserved.net->routes.front()).empty(),
          "cross-source repair removed the protected physical owner");
  require(ordinary.net->routes.front().owner == nullptr,
          "cross-source repair retained the conflicting ordinary owner");
  fpga::Tile *tile = fpga::Device::current().getTile(
      reserved_fragment.from.x, reserved_fragment.from.y);
  require(tile && (tile->cb.src.jump & bit(reserved_fragment.jump)) != NodeMask{},
          "repair released the protected owner's source lease");
}

void repeated_stage_entries_share_one_timeout_budget() {
  constexpr double budget = 300.0;
  double elapsed = 120.0;

  // Re-entering a logical stage keeps the time consumed by earlier passes.
  require(pnr::routeStageSecondsRemaining(elapsed, budget) == 180.0,
          "initial cumulative stage timeout accounting is incorrect");
  elapsed += 179.0;
  require(!pnr::routeStageTimeoutIsFatal(elapsed, budget, true),
          "stage timeout expired before its cumulative budget");
  elapsed += 1.0;
  require(pnr::routeStageTimeoutIsFatal(elapsed, budget, true),
          "stage re-entry incorrectly received a fresh timeout budget");
  require(!pnr::routeStageTimeoutIsFatal(elapsed, budget, false),
          "a completed stage was classified as timed out");

  // Scope cleanup must account an iteration that exits before pass reporting.
  double charged = 0.0;
  {
    pnr::RouteStageTimeCharge charge(charged);
  }
  require(charged > 0.0,
          "an early scheduler exit escaped cumulative timeout accounting");
}

} // namespace

int main() {
  try {
    releasing_one_conflicting_owner_preserves_the_survivor();
    non_owning_shared_prefix_does_not_leak_removed_owner_lease();
    cross_source_shared_prefix_requires_same_source_owner();
    packed_connection_transfers_prefix_ownership_across_rtl_nets();
    moving_parent_transfers_private_suffix_to_descendant();
    moving_parent_distributes_ownership_across_branch_depths();
    incomplete_owner_keeps_completed_shared_sibling_valid();
    randomized_shared_prefix_repairs_return_to_source_state();
    cross_source_private_node_conflict_repairs_both_trees();
    protected_owner_wins_a_cross_source_node_conflict();
    repeated_stage_entries_share_one_timeout_budget();
  } catch (const std::exception &error) {
    std::fprintf(stderr, "repair_prefixes_test failed: %s\n", error.what());
    return 1;
  }
  return 0;
}
