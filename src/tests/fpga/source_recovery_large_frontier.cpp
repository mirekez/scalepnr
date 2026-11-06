#include "SourceInputFixture.h"
#include "RepairStatistics.h"

#include <iostream>

// A captured Source Moving task retried its 524288-label recovery after 36
// transit cuts. The captured free connection needs 761408 expansions; no
// relocation or cut is required with a sufficient bounded wire search.
// Two synthetic binary trees reproduce broad frontiers without device names.
void source_recovery_large_frontier_regression()
{
    using routing_tests::SourceInputFixture;
    using routing_tests::require;
    constexpr int count = (1 << 19) - 1;
    constexpr int first_leaf = (count - 1) / 2;
    constexpr size_t enough = 2097152;
    const char* old_profile = std::getenv("SCALEPNR_PROFILE_REPAIR");
    struct RestoreProfile {
        const char* name;
        bool set;
        std::string value;
        ~RestoreProfile() {
            if (set) setenv(name,value.c_str(),1);
            else unsetenv(name);
        }
    } restore{"SCALEPNR_PROFILE_REPAIR",old_profile != nullptr, old_profile ? old_profile : ""};
    const char* old_cap=std::getenv("SCALEPNR_FREE_SEARCH_STATES");
    RestoreProfile restore_cap{"SCALEPNR_FREE_SEARCH_STATES",old_cap != nullptr,old_cap ? old_cap : ""};
    setenv("SCALEPNR_PROFILE_REPAIR","1",1);
    // Match the production runner's fast probe; include its work in the bound.
    setenv("SCALEPNR_FREE_SEARCH_STATES","65536",1);
    SourceInputFixture f(false, 0, 4, 288, 3);
    auto location = [](int tree, int n) {
        return fpga::Coord{24 + 2 * (n / CB_MAX_NODES) + tree, 0};
    };
    auto id = [](int n) { return n % CB_MAX_NODES; };
    for (int tree = 0; tree < 2; ++tree) {
        for (int n = 0; n < count; n += CB_MAX_NODES) {
            auto& tile = f.tile(location(tree, n));
            auto& cb = f.device.cb_types.emplace_back();
            cb.type_id = cb.base_type_id = f.device.cb_types.size() - 1;
            cb.name = "FILL_MATRIX_" + std::to_string(cb.type_id);
            tile.cb_type = tile.cb.type = &cb;
        }
    }
    f.edge({7,0}, location(0,0), 141, 3500, 3500, true);
    f.terminal(location(0,0), 3500, 0);
    for (int n = 0; n < first_leaf; ++n) {
        for (int child : {2*n+1, 2*n+2}) {
            f.resolvedLocal(location(0,n), id(n), location(0,child),
                child >= first_leaf ? fpga::CB_NODE_SRC : fpga::CB_NODE_LOCAL, id(child));
        }
    }
    for (int n = 1; n < count; ++n)
        f.resolvedLocal(location(1,n), id(n), location(1,(n-1)/2),
            fpga::CB_NODE_LOCAL, id((n-1)/2));
    f.resolvedLocal(location(1,0), 0, {0,0}, fpga::CB_NODE_LOCAL, 87);
    // Only the last leaf connects the trees. All earlier source leaves are
    // real routing outputs with no landing; reverse leaves have no driver.
    auto& from = f.tile(location(0,count-1));
    auto& to = f.tile(location(1,count-1));
    fpga::CBType::ResolvedJump jump;
    jump.delta = to.coord-from.coord;
    jump.target_tile_coord = true;
    jump.target_cb_type_id = to.cb_type->type_id;
    jump.dsts.jump.setBit(CB_MAX_NODES-1);
    from.cb_type->dst_by_src[id(count-1)].push_back(jump);
    SourceInputFixture::name(to, fpga::CB_NODE_DST, CB_MAX_NODES-1);
    to.cb_type->dst_local[CB_MAX_NODES-1].local.setBit(id(count-1));
    for (auto& cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();

    auto& router = f.tech.route;
    router.complete_routes_only = true;
    router.moving_stage = router.moving_sources_stage = true;
    router.route_preemption_enabled = router.route_deadends_enabled = false;
    pnr::RouteDesign::RouteTask task{f.source, f.sink, f.signal, "Q", "I0", "FILL_BROAD_RECOVERY"};
    task.endpoints_prepared = true;
    std::ostringstream control_before;
    fpga::dumpNetRouteHistory(*f.enable, control_before);
    // The original production bound must genuinely fail without publishing
    // partial leases. Otherwise this fixture does not reproduce the failure.
    fpga::resetRepairStatistics();
    require(!router.completeTaskFromSource(task, false, 524288, true, true),
        "broad free fixture did not exhaust the original recovery allowance");
    const auto old_work=fpga::repairStatistics();
    require(old_work[fpga::RepairCounter::source_expanded]>=524288 &&
            old_work[fpga::RepairCounter::source_limit]>0,
        "fixture failed before exercising the bounded search");
    require(f.sink->wires.empty(), "limited recovery published a partial route");
    f.auditAll();
    rtl::NetRouteBinding endpoints;
    endpoints.from=f.source; endpoints.to=f.sink;
    endpoints.from_port="Q"; endpoints.to_port="I0"; endpoints.route_name=task.net_name;
    auto reverse=router.backwardDockingIndex({0,0},288);
    fpga::RepairIncomingSources incoming=[&](auto coord,auto type,int node,const auto& visit) {
        pnr::visitBackwardPhysicalSources(reverse,coord,type,node,visit);
    };
    bool limited=false;
    const bool witness=fpga::routeUnboundNetFromSource(*f.signal,endpoints,
        SourceInputFixture::pin({7,0},141,true),SourceInputFixture::pin({0,0},87,false),
        {},enough,200,{},incoming,&limited);
    std::cout<<"FILL_BROAD_CONTROL complete="<<witness<<" limited="<<limited<<'\n';
    require(witness,"synthetic broad frontier has no complete low-level witness");
    require(fpga::unrouteNetRoute(*f.signal,0),"cannot release control witness");
    f.auditAll();
    // An explicit sufficient free-only allowance must not be silently clamped
    // back to the failed limit, nor replace a wire search with a relocation.
    fpga::resetRepairStatistics();
    const bool done = router.recoverMovingSourceWires(task,true);
    const auto work=fpga::repairStatistics();
    std::cout << "FILL_BROAD_RECOVERY nodes=" << 2*count << " old_budget=524288"
              << " requested_budget=" << enough << " complete=" << done
              << " old_expanded=" << old_work[fpga::RepairCounter::source_expanded]
              << " recovery_expanded=" << work[fpga::RepairCounter::source_expanded] << '\n';
    require(done && f.sink->wires.size() == 1 && fpga::isRouteComplete(f.sink->wires.front()),
        "free-only recovery discarded its explicit larger search allowance");
    // Bound the total work, including the unchanged smaller attempts: this
    // must be one escalation, not repeated copies of the million-node search.
    require(work[fpga::RepairCounter::source_expanded]>524288 &&
            work[fpga::RepairCounter::source_expanded]<1800000 &&
            work[fpga::RepairCounter::source_success]==1,
        "recovery repeated or bypassed the bounded free searches");
    require(f.source->tile.peer == &f.tile({7,0}) && f.sink->tile.peer == &f.tile({0,0}) &&
        router.route_stats.preempt_success == 0, "wire recovery changed placement or preempted an owner");
    std::ostringstream control_after;
    fpga::dumpNetRouteHistory(*f.enable, control_after);
    require(control_before.str() == control_after.str(), "wire recovery changed the existing control tree");
    f.auditAll();
}
