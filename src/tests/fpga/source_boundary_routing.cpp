#include "SourceInputFixture.h"

#include <iostream>

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
            // Releasing the transit exposes the long current-position route.
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
                // No short source placement can reach the old output.
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

            // Keep a nonempty isolated source component with converging branches.
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


bool runBoundaryCase(bool opens_current, bool allow_cuts, int fanout,
                     bool grows_component = false, bool final_takeoff = false,
                     int entry = 0) {
    BoundaryFixture fixture(opens_current,fanout,grows_component,final_takeoff);
    auto& f = fixture.f; auto& router = f.tech.route;
    // Example 1 reached Source routing from Destination Moving, after the
    // source had already completed a trunk. Keep the real caller's stage:
    // earlier fixed-sink coverage accidentally left Source Moving enabled.
    if (entry == 3) router.moving_sources_stage = false;
    // Boundary policy checks the owners' protection, independently of the
    // numeric solver's preemption switch. Exercise that real rejection path.
    for (auto* owner : fixture.victims) owner->route_protected = !allow_cuts;
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0",
                                    "boundary_routing_signal_97"};
    task.endpoints_prepared = true;
    pnr::MovingPlacementBlacklist placements({{f.source}});
    std::vector<pnr::RouteDesign::RouteTask> moved_tasks;
    std::string reason;
    size_t prefixes = 0, retries = 0;
    bool moved = false, retryable = false;
    require(fixture.partialOwners() == 0 && f.sink->wires.empty() &&
            fixture.availableNewExits() == 0,
            "boundary fixture started with changed ownership");
    f.auditAll();
    bool done;
    if (entry) {
        // The real post-batch deadline occurred through Generic task repair,
        // not the initial stage scheduler. Both it and fixed-destination
        // fallback must share the same retry policy and retained blacklist.
        router.preserve_routes = true;
        f.sink->pnr_completed = true;
        done = entry == 1 ? router.routeTaskOrFail(task,"Moving sources",2,true,true)
                          : router.moveUnfinishedDestination(task,&moved_tasks,&reason,true);
        if (entry == 3)
            require(!router.moving_sources_stage && !router.moving_focus_inst,
                    "fixed-sink source routing leaked Source stage or focus");
        moved = f.source->tile.peer != &f.tile({7,0});
        // These public entry points own their retry counter. Every partial
        // obstruction is an independently audited committed boundary cut.
        retries = fixture.partialOwners();
    } else {
        done = router.moveUnfinishedSource(task, &moved_tasks, &reason, &prefixes, &moved, &retryable, nullptr, false, &placements, &retries);
    }
    f.auditAll();
    std::cout << "Boundary routing current_opened=" << opens_current
        << " entry=" << entry
        << " cuts_allowed=" << allow_cuts << " fanout=" << fanout
        << " grows_component=" << grows_component
        << " final_takeoff=" << final_takeoff
        << " complete=" << done << " moved=" << moved << " releases=" << retries
        << " partial_owners=" << fixture.partialOwners()
        << " available_new_exits=" << fixture.availableNewExits()
        << " reason=" << reason << '\n';
    if (!allow_cuts) {
        require(!done && !moved &&
                fixture.partialOwners() == 0 && f.sink->wires.empty() &&
                f.source->tile.peer == &f.tile({7,0}),
                "refused boundary changed ownership or placement");
        return true;
    }
    const auto output = f.signal->findRouteBinding(task.from,task.to,task.from_port,
                                                  task.to_port,task.net_name);
    const auto* binding = output.index < f.signal->routes.size() ? &f.signal->routes[output.index] : nullptr;
    const auto* completed_route = binding && binding->owner &&
        binding->route_index < binding->owner->wires.size()
            ? &binding->owner->wires[binding->route_index] : nullptr;
    require(done && completed_route && fpga::isRouteComplete(*completed_route),
            "boundary fixture failed before reproducing successful routing: " + reason);
    std::ostringstream reachability;
    require(fpga::auditNetRouteReachability(*f.signal,output.index,reachability),
            "boundary route does not connect its physical endpoints: " + reachability.str());
    require(fixture.partialOwners() == size_t(opens_current ? (final_takeoff ? 2 : 1) : 3),
            "fixture did not release the required independent transit owners");
    if (opens_current) {
        require(f.source->tile.peer && moved == !same(f.source->coord,{7,0}),
                "routing reported a move inconsistent with the committed source placement");
    } else require(moved && f.source->tile.peer == &f.tile({1,0}),
                   "routing missed the only legal relocated source placement");
    bool ce_complete = false;
    for (size_t index = 0; index < f.enable->routes.size(); ++index) {
        const auto& ce = f.enable->routes[index];
        if (ce.from != f.control || ce.to != f.source || ce.to_port != "CE" ||
            !ce.owner || ce.route_index >= ce.owner->wires.size()) continue;
        const auto& route = ce.owner->wires[ce.route_index];
        if (!fpga::isRouteComplete(route) || route.empty() ||
            !same(route.back().to,f.source->coord)) continue;
        std::ostringstream input_reachability;
        ce_complete = fpga::auditNetRouteReachability(*f.enable,index,input_reachability);
        if (ce_complete) break;
    }
    require(ce_complete,"source relocation left its control input incomplete or disconnected");
    if (opens_current && !moved) {
        size_t hops = 0;
        for (const auto& wire : *completed_route)
            if (wire.type == fpga::Wire::WIRE_CROSSBAR && !same(wire.from,wire.to)) ++hops;
        require(hops > 10, "current-position route missed the physical long corridor");
    }
    if (final_takeoff)
        require(fixture.partialOwners() == 2 && router.route_stats.preempt_success >= 2,
                "final full routing failed to preempt the independent Takeoff obstruction");
    if (grows_component)
        require(fixture.availableNewExits() == 3,
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
    return true;
}
} // namespace

// Exercise owned transit cuts and the resulting physical routes through the
// production Source, Generic, fixed-destination, and Destination-stage entries.
void source_boundary_routing_regression() {
    require(runBoundaryCase(true,true,0), "current-position boundary route failed");
    require(runBoundaryCase(true,true,0,false,true),
            "independent Takeoff preemption failed");
    require(runBoundaryCase(false,false,8), "protected-owner control failed");
    const bool isolated = runBoundaryCase(false,true,32);
    const bool growing = runBoundaryCase(false,true,32,true);
    require(isolated && growing,
            "relocated boundary routes failed");
}

void source_boundary_generic_routing_regression() {
    require(runBoundaryCase(false,true,32,true,false,1),
            "Generic task repair missed the boundary route");
}

void source_boundary_fixed_routing_regression() {
    require(runBoundaryCase(false,true,32,true,false,2),
            "fixed-destination repair missed the boundary route");
}

void source_boundary_destination_stage_regression() {
    // The output's only legal new placement is behind three independently
    // owned completed transits. Each has a free repair bypass. Fixed placement
    // must stay fixed; the movable source must use the same boundary routing
    // as Source Moving, then restore the Destination stage. Names are abstract
    // and all busy nodes have registered real route owners.
    require(runBoundaryCase(false,true,32,true,false,3),
            "destination-stage source fallback missed the boundary route");
}
