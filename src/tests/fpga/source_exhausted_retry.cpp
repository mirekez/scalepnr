#include "SourceInputFixture.h"
#include "route/MovingPlacementBlacklist.h"

#include <iostream>

namespace {
using routing_tests::SourceInputFixture;
using routing_tests::require;

void finalize(SourceInputFixture& f) {
    for (auto& cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
}
} // namespace

// A failed source move must preserve the routed CE tree and all leases. Adding
// one real continuation to the same placed source must then complete the task.
void source_exhausted_budget_retry_regression() {
    SourceInputFixture f(false);
    // This checks a failed trace, not an input failure after a committed move.
    // Retain all transit resources but provide no alternate packing location.
    for (auto at : {fpga::Coord{1,0}, fpga::Coord{2,0}})
        f.tile(at).tile_type->elements.clear();
    auto& router = f.tech.route;
    router.moving_stage = router.moving_sources_stage = true;
    router.complete_routes_only = true;
    router.route_recursion_budget = 100;
    router.route_preemption_enabled = router.route_deadends_enabled = false;
    router.iteration_limit = router.route_suffix_depth_limit = 200;

    // Original source has a free, nonempty but disconnected forward component.
    // The sink has physically incoming alternatives, not a sealed LOCAL pin.
    f.edge({7,0}, {7,1}, 141, 3300, 3301, true);
    SourceInputFixture::name(f.tile({7,1}), fpga::CB_NODE_JOINT, 3302);
    f.tile({7,1}).cb_type->dst_joint[3301].joint.setBit(3302);
    // The alternate register placements cannot receive CE from its driver.
    // Keep the old, fully routed CE tree untouched at the current placement.
    f.tile({6,0}).cb_type->local_src[169].jump.clearBit(900);
    finalize(f);
    f.auditAll();

    std::vector<fpga::CBState> before;
    std::vector<NodeMask> pins_before;
    for (auto& tile : f.device.tile_grid) {
        before.push_back(tile.cb);
        pins_before.push_back(tile.pin_state.leased_nodes);
    }
    std::ostringstream control_before;
    fpga::dumpNetRouteHistory(*f.enable, control_before);
    auto unchanged = [&] {
        for (size_t i = 0; i < before.size(); ++i) {
            auto& tile = f.device.tile_grid[i];
            auto& cb = tile.cb;
            auto& old = before[i];
            require(cb.src.jump == old.src.jump && cb.dst.jump == old.dst.jump &&
                    cb.joint.jump == old.joint.jump && cb.local.local == old.local.local &&
                    tile.pin_state.leased_nodes == pins_before[i],
                    "failed source move changed leases at (" +
                        std::to_string(tile.coord.x) + "," + std::to_string(tile.coord.y) + ")");
            require(cb.src_deadend.jump == old.src_deadend.jump,
                    "failed source move changed persistent deadends at (" +
                        std::to_string(tile.coord.x) + "," + std::to_string(tile.coord.y) + ")");
        }
        std::ostringstream control_after;
        fpga::dumpNetRouteHistory(*f.enable, control_after);
        require(control_after.str() == control_before.str() && f.sink->wires.empty() &&
                f.source->tile.peer == &f.tile({7,0}) && f.source->pos == 0,
                "failed source probe changed CE ownership, placement or output routes");
        f.auditAll();
    };
    pnr::RouteDesign::RouteTask task{f.source, f.sink, f.signal, "Q", "I0",
                                    "exhausted_source_signal_83"};
    task.endpoints_prepared = true;
    pnr::MovingPlacementBlacklist blacklist({{f.source}});
    bool moved = false;
    std::string reason;
    require(!router.moveUnfinishedSource(task, nullptr, &reason, nullptr, &moved,
                nullptr, nullptr, false, &blacklist),
            "disconnected source unexpectedly found a CE-compatible relocation");
    unchanged();

    // Real occupancy/topology change: the old forward component now reaches
    // the input. The SAME task and placement blacklist must not hide it.
    f.edge({7,1}, {0,0}, 3301, 3303, 3304);
    f.terminal({0,0}, 3304, 87);
    finalize(f);
    // Production connectivity is immutable after loading. This fixture edits
    // it between calls, so rebuild the router's cached physical reverse graph.
    router.docking_resolved_arcs = {};
    router.docking_resolved_arcs_complete = false;
    require(router.moveUnfinishedSource(task, nullptr, &reason, nullptr, &moved,
                nullptr, nullptr, false, &blacklist) && !moved &&
            f.source->tile.peer == &f.tile({7,0}) && f.sink->wires.size() == 1 &&
            fpga::isRouteComplete(f.sink->wires.front()),
            "new physical continuation did not complete at the current placement");
    f.auditAll();
    std::ostringstream control_opened;
    fpga::dumpNetRouteHistory(*f.enable, control_opened);
    require(control_opened.str() == control_before.str(),
            "opened-path wire routing changed the retained CE route");

    std::cout << "Source continuation: failed attempt unchanged, opened path complete\n";
}
