#include "SourceInputFixture.h"
#include "route/MovingSourceRecovery.h"

#include <cstdio>
#include <iostream>
#include <set>

namespace {
using routing_tests::SourceInputFixture;
using routing_tests::require;
using fpga::Coord;
using Route = std::vector<fpga::Wire>;

bool same(Coord a, Coord b) { return a.x == b.x && a.y == b.y; }

struct BoundaryFixture {
    SourceInputFixture f{false, 0, 16};
    std::vector<rtl::Net*> victims;
    std::vector<Coord> middles;
    bool growing = false;

    BoundaryFixture(bool opens_current, int fanout, bool grows_component, bool final_takeoff) {
        growing = grows_component;
        // Remove the short alternate Q routes, without disturbing the existing
        // routed CE tree. Only the constructed corridor can reach the sink.
        f.tile({1,0}).cb_type->local_src[141].jump.clearBit(300);
        for (int n = 0; n < 32; ++n)
            f.tile({2,0}).cb_type->local_src[141].jump.clearBit(100+n);

        if (opens_current) {
            // The obstruction lies outside Grounding's window and after the
            // first landing, so wire-only Takeoff/Grounding cannot cut it.
            // Reverse relocation releases it; the NEXT current-position
            // search must notice the newly free, complete route.
            const std::vector<Coord> detour{{7,0},{6,0},{5,0},{4,0},{3,0},
                {2,0},{1,1},{0,1},{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{7,1}};
            int previous = 141;
            for (size_t i = 0; i+1 < detour.size(); ++i) {
                const int dst = i+2 == detour.size() ? 2200 : 3500+int(i);
                f.edge(detour[i],detour[i+1],previous,3500+int(i),dst,i == 0);
                previous = dst;
            }
            f.edge({1,0}, {7,1}, 141, 2000, 2200, true);
            f.edge({7,1}, {6,2}, 2200, 2001, 2100);
            f.edge({6,2}, {0,0}, 2100, 2002, 2800);
            f.terminal({0,0}, 2800, 87);
            owner(0, {6,2});
            if (final_takeoff) {
                // The reverse cut opens the long path, but its original
                // Takeoff still needs a separate legal transit preemption.
                // No short source placement can reach the old output, so
                // this must be recovered by the FINAL full wire attempt.
                f.tile({1,0}).cb_type->local_src[141].jump.clearBit(2000);
                owner(1,{7,0},true);
            }
        } else {
            // Three independently owned transit landings block a valid
            // relocation to (1,0). All cuts lie OUTSIDE the old source's
            // reachable component; none can make its current output useful.
            f.edge({1,0}, {1,2}, 141, 2000, 2100, true);
            f.edge({1,2}, {2,2}, 2100, 2001, 2101);
            f.edge({2,2}, {3,2}, 2101, 2002, 2102);
            f.edge({3,2}, {0,0}, 2102, 2003, 2800);
            f.terminal({0,0}, 2800, 87);
            for (int n = 0; n < 3; ++n) owner(n, {n+1,2});

            // A nonempty isolated source component with converging branches
            // makes repeated full wire recovery measurable by work, not time.
            f.edge({7,0}, {7,1}, 141, 3300, 3301, true);
            f.edge({7,1}, {7,2}, 3301, 3303, 3304);
            f.edge({7,2}, {7,1}, 3304, 3305, 3306);
            f.tile({7,1}).cb_type->dst_src[3306].jump.setBit(3303);
            for (int n = 0; n < fanout; ++n) {
                SourceInputFixture::name(f.tile({7,1}), fpga::CB_NODE_JOINT, 3400+n);
                f.tile({7,1}).cb_type->dst_joint[3301].joint.setBit(3400+n);
                f.tile({7,1}).cb_type->joint_src[3400+n].jump.setBit(3303);
            }
            if (grows_component) for (int n = 0; n < 3; ++n) {
                // Each reverse cut ALSO exposes a small new dead branch to
                // the old source. This is a genuine occupancy change in its
                // reachable graph, like the real failure, not an identical
                // snapshot that an ordinary failure blacklist can discard.
                // Entry is into the victim's outgoing SRC, never backwards
                // into the relocation corridor's incoming DST.
                const int local = 3560+n;
                SourceInputFixture::name(f.tile({7,1}),fpga::CB_NODE_LOCAL,local);
                f.tile({7,1}).cb_type->dst_local[3301].local.setBit(local);
                f.resolvedLocal({7,1},local,{n+1,2},fpga::CB_NODE_SRC,2420+n);
            }
        }
        for (auto& cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
        f.device.rebuildIncomingDstMasks();
        f.auditAll();
        auto& router = f.tech.route;
        router.moving_stage = router.moving_sources_stage = true;
        router.complete_routes_only = true;
        router.route_recursion_budget = 100;
        router.route_preemption_enabled = true;
        router.route_deadends_enabled = false;
        router.iteration_limit = router.route_suffix_depth_limit = 200;
    }

    void owner(int n, Coord obstruction, bool takeoff = false) {
        Coord driver{3+n,0}, middle{3+n,1}, sink{4+n,2};
        fpga::Element reg;
        reg.name = "obstruction_register_slot"; reg.type = fpga::ELEMENT_FD;
        f.tile(driver).tile_type->elements.push_back(reg);
        fpga::Element lut;
        lut.name = "obstruction_logic_slot"; lut.type = fpga::ELEMENT_LUT5;
        f.tile(sink).tile_type->elements.push_back(lut);
        lut.name = "obstruction_logic_extension"; lut.type = fpga::ELEMENT_LUT1;
        f.tile(sink).tile_type->elements.push_back(lut);
        auto* from = f.cell("obstruction_source_"+std::to_string(n), "FDRE",
                            {{"Q",true}}, driver);
        auto* to = f.cell("obstruction_sink_"+std::to_string(n), "LUT6",
                          {{"I0",false}}, sink);
        from->outline.fixed = to->outline.fixed = true;
        auto* net = f.connect(*from, "Q", *to, "I0");
        Route route{SourceInputFixture::pin(driver,141,true),
            f.edge(driver,middle,141,2400+n,2400+n,true),
            f.edge(middle,obstruction,2400+n,2410+n,2100+n)};
        if (takeoff) {
            route.push_back(f.edge(obstruction,{6,0},2100+n,3500,3500));
            route.push_back(f.edge({6,0},sink,3500,2420+n,2500+n));
        } else route.push_back(f.edge(obstruction,sink,2100+n,2420+n,2500+n));
        route.push_back(f.terminal(sink,2500+n,87));
        route.push_back(SourceInputFixture::pin(sink,87,false));
        // Each displaced owner also has a real free bypass. A successful
        // relocation must leave its retained prefix repairable through it.
        f.edge(middle,sink,2400+n,2430+n,2600+n);
        f.terminal(sink,2600+n,87);
        for (auto& cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
        f.device.rebuildIncomingDstMasks();
        require(pnr::commitPreparedRoute(route), "cannot commit owned transit obstruction");
        to->wires.push_back(std::move(route));
        const auto id = fpga::attachNetRoute(*net,*to,0,from,to,"Q","I0",net->name);
        fpga::registerNetRouteTiles(*net,to->wires.front(),id);
        victims.push_back(net); middles.push_back(middle);
    }

    size_t partialOwners() const {
        size_t partial = 0;
        for (auto* net : victims) {
            auto& binding = net->routes.front();
            auto& route = binding.owner->wires[binding.route_index];
            if (!fpga::isRouteComplete(route)) ++partial;
        }
        return partial;
    }
    size_t availableNewExits() const {
        size_t count = 0;
        if (growing) for (int n = 0; n < 3; ++n)
            if (!f.device.getTile(n+1,2)->cb.src.jump.testBit(2420+n)) ++count;
        return count;
    }
};

struct RecoveryTrace : fpga::RouteSearchTrace {
    BoundaryFixture& fixture;
    std::ostringstream output;
    uint64_t audited_revision = fpga::routeMutationRevision();
    size_t cut_audits = 0, maximum_partial = 0, maximum_new_exits = 0;
    explicit RecoveryTrace(BoundaryFixture& f) : fixture(f) {}
    std::ostream& stream() override { return output; }
    void snapshot(fpga::Tile& tile) override {
        const auto revision = fpga::routeMutationRevision();
        // Observe the next actual old-position search after each cut, outside
        // the temporary trial placements and the final relocation transaction.
        if (revision == audited_revision || !same(tile.coord,{7,0}) ||
                fixture.f.source->tile.peer != &fixture.f.tile({7,0})) return;
        audited_revision = revision;
        fixture.f.auditAll();
        ++cut_audits;
        maximum_partial = std::max(maximum_partial,fixture.partialOwners());
        maximum_new_exits = std::max(maximum_new_exits,fixture.availableNewExits());
    }
    size_t starts(bool two_ended_only = false) const {
        const auto text = output.str();
        size_t count = 0;
        for (const char* phase : {"source_forward_pop", "source_pop"}) {
            if (two_ended_only && std::string_view(phase) == "source_pop") continue;
            const std::string marker = "phase=" + std::string(phase) + " from=(7,0)/" +
                std::to_string(fpga::CB_NODE_LOCAL) + ":141 ";
            for (size_t pos = 0; (pos = text.find(marker,pos)) != std::string::npos;
                 pos += marker.size()) ++count;
        }
        return count;
    }
    size_t expansions(bool isolated_only = false, bool unique = false) const {
        size_t count = 0;
        std::set<std::tuple<int,int,int,int>> nodes;
        std::istringstream lines(output.str()); std::string line;
        while (std::getline(lines,line)) {
            if (line.find("phase=source_forward_pop ") == std::string::npos &&
                line.find("phase=source_reverse_pop ") == std::string::npos &&
                line.find("phase=source_pop ") == std::string::npos) continue;
            if (isolated_only) {
                const auto pos = line.find("from=");
                int x, y, kind, node;
                if (pos == std::string::npos || std::sscanf(line.c_str()+pos,
                        "from=(%d,%d)/%d:%d",&x,&y,&kind,&node) != 4) continue;
                // Track numeric identities, not just Tiles: the retained CE
                // tree also has an unrelated input landing in Tile (7,0).
                if (!(x == 7 && ((y == 0 && (node == 141 || node == 3300)) ||
                        (y == 1 && (node == 3301 || node == 3303 || node == 3306 ||
                                    (node >= 3400 && node < 3464) ||
                                    (node >= 3560 && node < 3563))) ||
                        (y == 2 && (node == 3304 || node == 3305))))) continue;
                if (unique && !nodes.emplace(x,y,kind,node).second) continue;
            }
            ++count;
        }
        return count;
    }
    size_t exposedExits() const {
        size_t count = 0;
        const auto text = output.str();
        for (int n = 0; n < 3; ++n) {
            const auto marker = "from=(" + std::to_string(n+1) + ",2)/" +
                std::to_string(fpga::CB_NODE_SRC) + ":" + std::to_string(2420+n) + " ";
            std::istringstream lines(text); std::string line;
            while (std::getline(lines,line))
                if ((line.find("phase=source_forward_pop ") != std::string::npos ||
                     line.find("phase=source_pop ") != std::string::npos) &&
                    line.find(marker) != std::string::npos) { ++count; break; }
        }
        return count;
    }
    size_t wireCalls(bool free_only) const {
        const auto text = output.str();
        const std::string marker = "MOVING_SOURCE_WIRE_RECOVERY free_only=" +
            std::to_string(int(free_only));
        size_t count = 0;
        for (size_t pos = 0; (pos = text.find(marker,pos)) != std::string::npos;
             pos += marker.size()) ++count;
        return count;
    }
};

bool runBoundaryCase(bool opens_current, bool allow_cuts, int fanout,
                     bool grows_component = false, bool final_takeoff = false,
                     int entry = 0) {
    BoundaryFixture fixture(opens_current,fanout,grows_component,final_takeoff);
    auto& f = fixture.f; auto& router = f.tech.route;
    // Boundary policy checks the owners' protection, independently of the
    // numeric solver's preemption switch. Exercise that real rejection path.
    for (auto* owner : fixture.victims) owner->route_protected = !allow_cuts;
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0",
                                    "boundary_recovery_signal_97"};
    task.endpoints_prepared = true;
    pnr::MovingPlacementBlacklist placements({{f.source}});
    std::vector<pnr::RouteDesign::RouteTask> moved_tasks;
    std::string reason;
    size_t prefixes = 0, retries = 0;
    bool moved = false, retryable = false;
    size_t baseline_starts, baseline_expanded, baseline_nodes;
    {
        RecoveryTrace baseline(fixture);
        require(!router.completeTaskFromSource(task,false,
                    fpga::UNBOUND_ROUTE_STATE_BUDGETS.back(),true),
                "baseline unexpectedly bypassed the transit boundaries");
        baseline_starts = baseline.starts();
        baseline_expanded = baseline.expansions(true);
        baseline_nodes = baseline.expansions(true,true);
    }
    require(baseline_starts > 0 && fixture.partialOwners() == 0 && f.sink->wires.empty() &&
            fixture.availableNewExits() == 0,
            "baseline failed to measure read-only current-position recovery");
    f.auditAll();
    RecoveryTrace trace(fixture);
    bool done;
    if (entry) {
        // The real post-batch deadline occurred through Generic task repair,
        // not the initial stage scheduler. Both it and fixed-destination
        // fallback must share the same retry policy and retained blacklist.
        router.preserve_routes = true;
        f.sink->pnr_completed = true;
        done = entry == 1 ? router.routeTaskOrFail(task,"Moving sources",2,true,true)
                          : router.moveUnfinishedDestination(task,&moved_tasks,&reason,true);
        moved = f.source->tile.peer != &f.tile({7,0});
        // These public entry points own their retry counter. Every partial
        // obstruction is an independently audited committed boundary cut.
        retries = fixture.partialOwners();
    } else {
        done = pnr::recoverMovingSourceBoundaries(router,task,placements,
            &moved_tasks,reason,prefixes,moved,retryable,retries);
    }
    const auto starts = trace.starts(), expanded = trace.expansions();
    const auto old_expanded = trace.expansions(true);
    f.auditAll();
    std::cout << "Boundary recovery current_opened=" << opens_current
        << " entry=" << entry
        << " cuts_allowed=" << allow_cuts << " fanout=" << fanout
        << " grows_component=" << grows_component
        << " final_takeoff=" << final_takeoff
        << " complete=" << done << " moved=" << moved << " releases=" << retries
        << " baseline(starts=" << baseline_starts << ",old_expanded=" << baseline_expanded
        << ",old_nodes=" << baseline_nodes << ") old_position_starts=" << starts
        << " two_ended=" << trace.starts(true)
        << " old_component_expanded=" << old_expanded << " expanded=" << expanded
        << " exposed_exits=" << trace.exposedExits()
        << " available_new_exits=" << trace.maximum_new_exits
        << " full_wire_calls=" << trace.wireCalls(false)
        << " free_wire_calls=" << trace.wireCalls(true)
        << " audited_cuts=" << trace.cut_audits << " reason=" << reason << '\n';
    if (!allow_cuts) {
        require(!done && !moved && retries == 0 && starts == baseline_starts &&
                fixture.partialOwners() == 0 && f.sink->wires.empty() &&
                f.source->tile.peer == &f.tile({7,0}),
                "refused boundary changed ownership or repeated source recovery");
        return true;
    }
    const auto output = f.signal->findRouteBinding(task.from,task.to,task.from_port,
                                                  task.to_port,task.net_name);
    const auto* binding = output.index < f.signal->routes.size() ? &f.signal->routes[output.index] : nullptr;
    const auto* completed_route = binding && binding->owner &&
        binding->route_index < binding->owner->wires.size()
            ? &binding->owner->wires[binding->route_index] : nullptr;
    require(done && completed_route && fpga::isRouteComplete(*completed_route),
            "boundary fixture failed before reproducing successful recovery: " + reason);
    require(retries == size_t(opens_current ? 1 : 3) && trace.cut_audits >= retries &&
            trace.maximum_partial >= retries,
            "fixture did not exercise and audit every independent transit cut");
    require(moved == !opens_current && f.source->tile.peer == &f.tile(opens_current ? Coord{7,0} : Coord{1,0}),
            "recovery missed the required current-position or relocated solution");
    if (opens_current) {
        require(trace.wireCalls(true) == size_t(final_takeoff ? 0 : 1),
                "source reachability skipped a productive free cut or retried a blocked Takeoff");
        size_t hops = 0;
        for (const auto& wire : *completed_route)
            if (wire.type == fpga::Wire::WIRE_CROSSBAR && !same(wire.from,wire.to)) ++hops;
        require(hops > size_t(pnr::movingSourceSearchDepth()),
                "current-position control could be rescued by the short relocation walk");
    }
    if (final_takeoff)
        require(fixture.partialOwners() == 2 && router.route_stats.preempt_success >= 2,
                "final full recovery failed to preempt the independent Takeoff obstruction");
    if (grows_component)
        require(trace.maximum_new_exits == 3,
                "cuts failed to release the expected new old-source branches");
    for (size_t n = 0; n < fixture.victims.size(); ++n) {
        auto* net = fixture.victims[n];
        const auto owners = fpga::findNetOwnersByNode(f.tile(fixture.middles[n]),
                                                     fpga::CB_NODE_DST,2400+int(n));
        require(std::any_of(owners.begin(),owners.end(),[&](const auto& o){return o.net == net;}),
                "transit cut discarded the retained Takeoff landing");
        require(fpga::repairNetRouteGaps(*net,0), "displaced transit cannot use its free bypass");
        f.auditAll();
    }
    // These cuts open only dead branches at the old source. Neither public
    // dispatcher may restart its exact wire solver for those changes.
    if (entry)
        return trace.wireCalls(false) == size_t(entry == 1 ? 0 : 1) && trace.wireCalls(true) == 0;
    return opens_current ? (starts <= (final_takeoff ? 2*baseline_starts+retries : baseline_starts+1))
                         : (starts <= baseline_starts+retries &&
                            old_expanded <= baseline_expanded+retries*baseline_nodes);
}
} // namespace

// Example 1's 38 boundary releases restarted complete current-position recovery
// 39 times (804 low-level searches). This scaled replay retains the important
// distinction: cuts must not restart all completion modes and budgets,
// including cuts that expose small new unproductive source branches.
// A cut opening a real >10-hop current-position route MUST still be seen.
// The production scheduler calls the very same helper. No test-owned retry
// loop, simulated occupancy bit, time-based assertion, or external rescue.
void source_boundary_recovery_regression() {
    require(runBoundaryCase(true,true,0), "current-position control repeated full recovery");
    require(runBoundaryCase(true,true,0,false,true),
            "final preempting recovery repeated full recovery after each boundary");
    require(runBoundaryCase(false,false,8), "protected-owner control repeated full recovery");
    const bool isolated = runBoundaryCase(false,true,32);
    const bool growing = runBoundaryCase(false,true,32,true);
    require(isolated && growing,
            "boundary loop restarted full old-position recovery after transit cuts");
}

void source_boundary_generic_recovery_regression() {
    require(runBoundaryCase(false,true,32,true,false,1),
            "Generic task repair restarted full Source Moving recovery after boundary cuts");
}

void source_boundary_fixed_recovery_regression() {
    require(runBoundaryCase(false,true,32,true,false,2),
            "fixed-destination repair restarted full Source Moving recovery after boundary cuts");
}
