#include "SourceInputFixture.h"
#include "route/MovingSourceReachability.h"

#include <iostream>

void source_reachability_budget_regression() {
    using routing_tests::SourceInputFixture;
    using routing_tests::require;
    using fpga::Coord;
    // Example 1's free source component contained 2,909,330 nodes, but this
    // necessary-condition walk stopped at 524,288 and stayed UNKNOWN. Each
    // subsequent transit cut then retried exact wire recovery. Reproduce a
    // finite disconnected component just beyond that cap, with the only
    // useful exit blocked by a real control transit, not a manufactured bit.
    SourceInputFixture f(false,135);
    const Coord origin{7,0}, goal{0,0};
    f.tile(goal).cb_type->local_local[77].local.clearBit(115);
    // Leave the captured dedicated approach disconnected from the input.
    // Only the control transit's DST can supply the new useful output path.
    f.edge(origin,goal,141,4095,717,true);
    f.terminal(goal,717,115);
    std::vector<Coord> dead;
    for(int y=0;y<18;++y) for(int x=0;x<8;++x)
        if(!(x==goal.x && y==goal.y) && !(x==origin.x && y==origin.y))
            dead.push_back({x,y});
    dead.resize(130);
    for(Coord at:dead)
        for(int lane=2048;lane<4095;++lane)
            f.tile(at).cb_type->dst_by_src[lane].clear();
    for(int lane=2048;lane<4095;++lane) {
        f.edge(origin,dead.front(),141,lane,lane,true);
        for(size_t n=0;n+1<dead.size();++n)
            f.edge(dead[n],dead[n+1],lane,lane,lane);
    }
    // A legal control bypass lets the displaced owner remain fully routed
    // after the signal acquires the old transit arrival.
    f.edge({6,1},origin,710,1900,718);
    for(auto& cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
    f.auditAll();

    pnr::MovingSourceReachability region;
    region.target(&f.tile(goal),115);
    region.source(&f.tile(origin),141);
    require(!region.mayConnect([]{return false;}),
            "finite disconnected source component stayed UNKNOWN at the routing-state cap");
    const size_t examined=region.expanded();
    require(examined>fpga::UNBOUND_ROUTE_STATE_BUDGETS.back(),
            "fixture did not exceed the old necessary-condition cap");
    for(int n=0;n<100;++n) {
        region.released({{{5,17},fpga::CB_NODE_DST,4095}});
        require(!region.mayConnect([]{return false;}) && region.expanded()==examined,
                "unchanged large negative component repeated its walk");
    }
    const std::vector<fpga::RouteCutNode> cuts{{goal,fpga::CB_NODE_DST,717}};
    require(fpga::canPreemptNetRouteFromNodes(*f.enable,0,cuts),
            "large-component blocker was not an eligible control transit");
    auto released=fpga::previewNetRouteCuts({{f.enable,0}},cuts,true);
    require(fpga::unrouteNetRoutesFromNodes({{f.enable,0}},cuts,true),
            "cannot release the real terminal-approach blocker");
    region.released(released);
    require(region.mayConnect([]{return false;}) && region.expanded()<=examined+4,
            "large negative proof hid the newly freed useful exit");
    f.auditAll();
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","large_component_signal"};
    rtl::NetRouteBinding endpoints;
    endpoints.from=f.source; endpoints.to=f.sink;
    endpoints.from_port="Q"; endpoints.to_port="I0"; endpoints.route_name=task.net_name;
    require(fpga::routeUnboundNetFromSource(*f.signal,endpoints,
                SourceInputFixture::pin(origin,141,true),SourceInputFixture::pin(goal,115,false),
                {},32768,200),
            "positive necessary condition did not retain exact output validation");
    require(fpga::repairNetRouteGaps(*f.enable,0),
            "preempted control transit cannot use its free bypass");
    f.auditAll();
    std::cout << "Large source component expanded=" << examined
              << " unchanged_retries=100 output_complete=1 control_repaired=1\n";
}
