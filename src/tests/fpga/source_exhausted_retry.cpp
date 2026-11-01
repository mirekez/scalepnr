#include "SourceInputFixture.h"
#include "route/MovingPlacementBlacklist.h"

#include <iostream>

namespace {
using routing_tests::SourceInputFixture;
using routing_tests::require;

size_t occurrences(const std::string& text, const std::string& word) {
    size_t count = 0;
    for (size_t pos = 0; (pos = text.find(word, pos)) != std::string::npos;
         pos += word.size()) ++count;
    return count;
}

struct SearchTrace : fpga::RouteSearchTrace {
    std::ostringstream output;
    std::ostream& stream() override { return output; }
    void snapshot(fpga::Tile&) override {}
    size_t forwardStarts() const {
        return occurrences(output.str(), "phase=source_forward_pop from=(7,0)/" +
            std::to_string(fpga::CB_NODE_LOCAL) + ":141 ");
    }
    size_t expansions() const {
        const auto text = output.str();
        return occurrences(text, "phase=source_forward_pop ") +
            occurrences(text, "phase=source_reverse_pop ") +
            occurrences(text, "phase=source_pop ");
    }
};

void finalize(SourceInputFixture& f) {
    for (auto& cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
}
} // namespace

// Example 1 chunk 5 spent its Moving Sources deadline retrying an exhausted
// current-position search after reverse-boundary changes. Its free two-ended
// probe expanded the same 26775 states at 32K, 64K, 128K and 512K budgets.
// This small physical graph reproduces that inner restart without waiting ten
// minutes: both frontiers exhaust far below the SMALLEST production budget.
// There is a real placed register with a routed CE tree and real mapped pins;
// all occupied resources belong to that tree. Node names are abstract.
//
// Call the production budget loop and Moving entry, not a test copy of either.
// A negative search must stay read-only. Opening one physical continuation
// afterwards must still allow recovery: suppressing every retry is not a fix.
void source_exhausted_budget_retry_regression() {
    SourceInputFixture f(false);
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
            auto& cb = f.device.tile_grid[i].cb;
            auto& old = before[i];
            require(cb.src.jump == old.src.jump && cb.dst.jump == old.dst.jump &&
                    cb.joint.jump == old.joint.jump && cb.local.local == old.local.local &&
                    cb.src_deadend.jump == old.src_deadend.jump &&
                    f.device.tile_grid[i].pin_state.leased_nodes == pins_before[i],
                    "exhausted source probe changed live congestion");
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
    rtl::NetRouteBinding endpoints;
    endpoints.from = f.source; endpoints.to = f.sink;
    endpoints.from_port = "Q"; endpoints.to_port = "I0";
    endpoints.route_name = task.net_name;
    const auto output = SourceInputFixture::pin({7,0}, 141, true);
    const auto input = SourceInputFixture::pin({0,0}, 87, false);
    auto index = router.backwardDockingIndex({0,0}, 8);
    fpga::RepairIncomingSources incoming = [&](fpga::Coord at, fpga::CBNodeNameType type,
            int node, const fpga::RepairIncomingSourceVisitor& visit) {
        pnr::visitBackwardPhysicalSources(index, at, type, node, visit);
    };
    bool limited = false;
    require(!fpga::routeUnboundNetFromSource(*f.signal, endpoints, output, input,
                {}, 2, 200, {}, incoming, &limited) && limited,
            "a genuinely budget-limited frontier was reported as exhausted");
    require(!fpga::routeUnboundNetFromSource(*f.signal, endpoints, output, input,
                {}, fpga::UNBOUND_ROUTE_STATE_BUDGETS.front(), 200, {}, incoming,
                &limited) && !limited,
            "an exhausted frontier was reported as needing more budget");
    unchanged();
    size_t small_starts, small_expansions, full_starts, full_expansions;
    {
        SearchTrace trace;
        require(!router.completeTaskFromSource(task, false,
                    fpga::UNBOUND_ROUTE_STATE_BUDGETS.front(), true),
                "disconnected smallest-budget source search unexpectedly routed");
        small_starts = trace.forwardStarts();
        small_expansions = trace.expansions();
        require(trace.output.str().find("result=state_limit") == std::string::npos,
                "smallest-budget fixture hit a limit rather than exhausting its graph");
    }
    unchanged();
    {
        SearchTrace trace;
        require(!router.completeTaskFromSource(task, false,
                    fpga::UNBOUND_ROUTE_STATE_BUDGETS.back(), true),
                "disconnected full-budget source search unexpectedly routed");
        full_starts = trace.forwardStarts();
        full_expansions = trace.expansions();
    }
    unchanged();
    require(small_starts > 0 && small_expansions > 0 && small_expansions < 1000,
            "fixture did not exercise a bounded, exhausted physical frontier");
    pnr::MovingPlacementBlacklist blacklist({{f.source}});
    bool moved = false;
    std::string reason;
    size_t moving_starts, moving_expansions;
    {
        SearchTrace trace;
        require(!router.moveUnfinishedSource(task, nullptr, &reason, nullptr, &moved,
                    nullptr, nullptr, false, &blacklist),
                "disconnected source unexpectedly found a CE-compatible relocation");
        moving_starts = trace.forwardStarts();
        moving_expansions = trace.expansions();
    }
    unchanged();

    // Real occupancy/topology change: the old forward component now reaches
    // the input. The SAME task and placement blacklist must not hide it.
    f.edge({7,1}, {0,0}, 3301, 3303, 3304);
    f.terminal({0,0}, 3304, 87);
    finalize(f);
    require(router.moveUnfinishedSource(task, nullptr, &reason, nullptr, &moved,
                nullptr, nullptr, false, &blacklist) && !moved &&
            f.source->tile.peer == &f.tile({7,0}) && f.sink->wires.size() == 1 &&
            fpga::isRouteComplete(f.sink->wires.front()),
            "new physical continuation was suppressed by an exhausted-search result");
    f.auditAll();
    std::ostringstream control_opened;
    fpga::dumpNetRouteHistory(*f.enable, control_opened);
    require(control_opened.str() == control_before.str(),
            "opened-path wire recovery changed the retained CE route");

    std::cout << "Exhausted source work: smallest(starts=" << small_starts
        << ",expanded=" << small_expansions << ") full(starts=" << full_starts
        << ",expanded=" << full_expansions << ") moving(starts=" << moving_starts
        << ",expanded=" << moving_expansions << ") opened_path=complete\n";
    require(full_starts <= small_starts && full_expansions <= small_expansions &&
            moving_starts <= small_starts,
            "Source Moving repeats an already exhausted free frontier at larger budgets");
}
