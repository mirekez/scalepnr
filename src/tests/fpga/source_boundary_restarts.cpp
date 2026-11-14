#include "SourceInputFixture.h"
#include "Docking.h"
#include "PinLookupCache.h"

#include <iostream>
#include <streambuf>

namespace {
using routing_tests::SourceInputFixture;
using routing_tests::require;
using fpga::Coord;
using Route = std::vector<fpga::Wire>;

// Keep a dense set of independently owned transit boundaries, a CE-constrained
// register, and a legal relocation. The graph uses abstract numeric nodes.
constexpr int boundaries = 260;
constexpr int reverse_width = 128;

struct RestartFixture {
    SourceInputFixture f{false, 0, boundaries + 4};
    std::vector<rtl::Net*> victims;

    explicit RestartFixture(bool repeated_packable = false, bool repeated_input = false, bool growing_input = false) {
        f.tile({1,0}).cb_type->local_src[141].jump.clearBit(300);
        for (int n = 0; n < 32; ++n)
            f.tile({2,0}).cb_type->local_src[141].jump.clearBit(100+n);

        if (repeated_input || growing_input) {
            // This bank has a real data exit, but its CE ingress is disconnected
            // from the fixed control driver.
            f.tile({2,0}).cb_type->local_src[141].jump.setBit(100);
        }

        std::vector<Coord> banks;
        for (int y = 0; y < 3; ++y) for (int x = 0; x < 8; ++x) {
            if (y == 0 && (x == 0 || x == 1 || x == 2 || x == 6 || x == 7)) continue;
            // A cut in an owner's Takeoff/Grounding CB is protected. Keep
            // both endpoints away from the two transit CBs in this replay.
            if (y == 2 && (x == 3 || x == 4)) continue;
            banks.push_back({x,y});
            for (auto type : {fpga::ELEMENT_FD,fpga::ELEMENT_LUT5,fpga::ELEMENT_LUT1})
                for (int bit = 0; bit < 16; ++bit) {
                    fpga::Element e;
                    e.name = "owner_slot_"+std::to_string(int(type))+"_"+std::to_string(bit);
                    e.type = type; e.bitmap_pos = bit;
                    f.tile({x,y}).tile_type->elements.push_back(e);
                }
        }

        std::vector<Route> routes;
        std::vector<rtl::Inst*> drivers, sinks;
        for (int n = 0; n < boundaries; ++n) {
            const Coord driver = banks[n % banks.size()];
            const Coord sink = banks[(n+7) % banks.size()];
            const int lane = n / banks.size();
            const int fd_pos = (lane/8)*128 + ((lane/4)%2)*64 + (lane%4)*4;
            const int lut_pos = (lane/4)*128 + (lane%4)*4 + 3;
            auto& driver_tile = f.tile(driver);
            auto& sink_tile = f.tile(sink);
            driver_tile.tile_type->pin_map.output_nodes[
                driver_tile.getNodeNum("FDRE","Q",fd_pos)].setBit(500+n);
            driver_tile.cb_type->local_output_nodes.setBit(500+n);
            sink_tile.tile_type->pin_map.nodes[{"LUT6","I0",lut_pos}].setBit(2700+n);
            sink_tile.cb_type->local_input_nodes.setBit(2700+n);
            auto* from = f.cell("boundary_driver_"+std::to_string(n),"FDRE",
                                {{"Q",true}},driver,fd_pos);
            auto* to = f.cell("boundary_receiver_"+std::to_string(n),"LUT6",
                              {{"I0",false}},sink,lut_pos);
            from->outline.fixed = to->outline.fixed = true;
            auto* net = f.connect(*from,"Q",*to,"I0");
            victims.push_back(net);
            drivers.push_back(from); sinks.push_back(to);
            // The obstruction is a transit arrival, not a Takeoff/Grounding
            // resource. Its retained Takeoff and a real bypass remain usable.
            routes.push_back({SourceInputFixture::pin(driver,500+n,true),
                f.edge(driver,{3,2},500+n,100+n,1000+n,true),
                f.edge({3,2},{4,2},1000+n,1300+n,1700+n),
                f.edge({4,2},sink,1700+n,2000+n,2300+n),
                f.terminal(sink,2300+n,2700+n),
                SourceInputFixture::pin(sink,2700+n,false)});
            f.edge({3,2},sink,1000+n,3000+n,3300+n);
            f.terminal(sink,3300+n,2700+n);
            // Every cut also exposes a dead exit at the old source.
            f.tile({7,1}).cb_type->dst_local[3901].local.setBit(3600+n);
            f.resolvedLocal({7,1},3600+n,{4,2},fpga::CB_NODE_SRC,2000+n);
            if (growing_input) {
                // Each cut also opens a CE predecessor, while the retained
                // foreign Takeoff still blocks its continuation.
                f.edge({4,2},{3,0},1700+n,3300+n,650);
            }
        }

        // All 260 arrivals join one free suffix. Only the last arrival has a
        // Takeoff at the CE-compatible replacement register location (1,0).
        f.edge({4,2},{0,0},1700,2900,2800);
        for (int n = 1; n < boundaries; ++n)
            f.tile({4,2}).cb_type->dst_src[1700+n].jump.setBit(2900);
        f.terminal({0,0},2800,87);
        f.edge({1,0},{4,2},141,600,1700+boundaries-1,true);
        if (repeated_packable) {
            // This bank packs the source and its CE, but its early exit is
            // driven by no register output.
            f.edge({1,0},{4,2},3999,601,3600,true);
            f.tile({4,2}).cb_type->dst_src[3600].jump.setBit(2900);
        }

        // A free reverse component is unrelated to every cut; no source pin
        // enters its predecessor leaves.
        f.edge({5,2},{0,0},3650,2900,2800);
        for (int n = 0; n < reverse_width; ++n) {
            f.tile({5,2}).cb_type->dst_src[3650+n].jump.setBit(2900);
            f.edge({6,2},{5,2},3900+n,3650+n,3650+n);
        }
        // A converging old-source component is unchanged across cuts.
        f.edge({7,0},{7,1},141,3900,3901,true);
        f.edge({7,1},{7,2},3901,3902,3903);
        for (int n = 0; n < 32; ++n) {
            SourceInputFixture::name(f.tile({7,1}),fpga::CB_NODE_JOINT,3950+n);
            f.tile({7,1}).cb_type->dst_joint[3901].joint.setBit(3950+n);
            f.tile({7,1}).cb_type->joint_src[3950+n].jump.setBit(3902);
        }
        for (auto& cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
        f.device.rebuildIncomingDstMasks();
        for (int n = 0; n < boundaries; ++n) {
            auto* net = victims[n];
            auto* driver = drivers[n];
            auto* sink = sinks[n];
            require(pnr::commitPreparedRoute(routes[n]),"cannot commit transit owner");
            sink->wires.push_back(std::move(routes[n]));
            const auto id = fpga::attachNetRoute(*net,*sink,0,driver,sink,"Q","I0",net->name);
            fpga::registerNetRouteTiles(*net,sink->wires.front(),id);
        }
        f.auditAll();
        auto& router = f.tech.route;
        router.moving_stage = router.moving_sources_stage = true;
        router.complete_routes_only = router.preserve_routes = true;
        router.route_recursion_budget = 100;
        router.route_preemption_enabled = true;
        router.route_deadends_enabled = false;
        router.iteration_limit = router.route_suffix_depth_limit = 200;
        f.sink->pnr_completed = true;
    }
};

struct ReuseCounter : std::streambuf {
    std::string line;
    size_t packing_previews = 0;
    size_t pin_hits = 0;
    size_t pin_misses = 0;

    int overflow(int ch) override {
        if (ch == traits_type::eof()) return traits_type::not_eof(ch);
        if (ch == '\n') {
            if (line.starts_with("SOURCE_PACKING_PREVIEW tile=(1,0)"))
                ++packing_previews;
            if (auto* cache = fpga::PinLookupCache::active()) {
                pin_hits = std::max(pin_hits, cache->hits);
                pin_misses = std::max(pin_misses, cache->misses);
            }
            line.clear();
        } else line.push_back(char(ch));
        return ch;
    }
};

} // namespace

static void run_source_boundary_restarts(bool repeated_packable, bool repeated_input = false,
                                         bool growing_input = false, bool inspect_pin_lookups = false) {
    if (inspect_pin_lookups)
        require(!fpga::PinLookupCache::active(), "source routing inherited an endpoint lookup scope");
    RestartFixture fixture(repeated_packable, repeated_input, growing_input);
    auto& f = fixture.f;
    auto& router = f.tech.route;
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","boundary_restart_signal"};
    task.endpoints_prepared = true;
    for (auto* net : fixture.victims) {
        const auto& binding = net->routes.front();
        require(fpga::isRouteComplete(binding.owner->wires[binding.route_index]),
                "transit owner was incomplete before source routing");
    }
    f.auditAll();
    ReuseCounter reuse;
    std::ostream output(&reuse);
    {
        pnr::RouteCongestionTrace trace(task.net_name,"boundary_route",{},&output);
        require(router.routeTaskOrFail(task,"Moving sources",2,true,true),
                "known CE-compatible source relocation was not completed");
    }
    f.auditAll();
    const auto ref = f.signal->findRouteBinding(f.source,f.sink,"Q","I0",task.net_name);
    require(ref.index < f.signal->routes.size(),"requester has no completed binding");
    const auto& binding = f.signal->routes[ref.index];
    require(binding.owner && binding.route_index < binding.owner->wires.size() &&
            fpga::isRouteComplete(binding.owner->wires[binding.route_index]),
            "requester's replacement route is incomplete");
    std::ostringstream reachability;
    require(fpga::auditNetRouteReachability(*f.signal,ref.index,reachability),
            "replacement does not connect its real physical pins: "+reachability.str());
    bool enable_complete = false;
    for (const auto& b : f.enable->routes) {
        if (b.from != f.control || b.to != f.source || b.to_port != "CE" ||
                !b.owner || b.route_index >= b.owner->wires.size()) continue;
        const auto& route = b.owner->wires[b.route_index];
        enable_complete |= fpga::isRouteComplete(route) && !route.empty() &&
            route.back().to.x == 1 && route.back().to.y == 0;
    }
    require(enable_complete,"relocated register lost its real CE route");
    require(f.source->tile.peer == &f.tile({1,0}),"replay missed the CE-compatible relocation");
    size_t partial = 0;
    for (int n = 0; n < boundaries; ++n) {
        auto* net = fixture.victims[n];
        const auto& owner = net->routes.front();
        const auto& route = owner.owner->wires[owner.route_index];
        if (fpga::isRouteComplete(route)) continue;
        ++partial;
        require(!f.tile({4,2}).cb.src.jump.testBit(2000+n),
                "cut did not release its occupied transit");
        const auto owners = fpga::findNetOwnersByNode(f.tile({3,2}),fpga::CB_NODE_DST,1000+n);
        require(std::any_of(owners.begin(),owners.end(),[&](const auto& held){return held.net == net;}),
                "cut discarded an owner's retained Takeoff landing");
        require(fpga::repairNetRouteGaps(*net,0),"cut owner lost its real free bypass");
    }
    require(partial > 0,"source route did not exercise an owned transit cut");
    f.auditAll();
    if (repeated_packable)
        require(reuse.packing_previews > 0 && reuse.packing_previews <= 2,
                "unchanged source bank was repeatedly packed");
    if (inspect_pin_lookups)
        require(!fpga::PinLookupCache::active() && reuse.pin_misses > 0 &&
                reuse.pin_hits >= 2 * reuse.pin_misses,
                "source routing did not reuse physical pin lookups");
    std::cout << "Boundary route: cut_owners=" << partial << " source=("
              << f.source->coord.x << ',' << f.source->coord.y
              << ") packing_previews=" << reuse.packing_previews
              << " pin_hits=" << reuse.pin_hits << " pin_misses=" << reuse.pin_misses
              << " complete=1\n";
}

void source_boundary_restarts_regression() {
    run_source_boundary_restarts(false);
}

void source_boundary_packing_reuse_regression() {
    run_source_boundary_restarts(true);
}

void source_pin_lookup_reuse_regression() {
    // Endpoint lookup scope must close after the completed source route.
    run_source_boundary_restarts(true,false,false,true);
}

void source_boundary_input_reuse_regression() {
    run_source_boundary_restarts(false,true);
}

// A real owned transit blocks the CE backward path. Releasing that owner
// exposes the path from the fixed control source without changing topology.
void source_input_dependency_invalidation_regression() {
    SourceInputFixture f(false);
    auto& router=f.tech.route;
    for (const auto [at,type] : std::vector<std::pair<Coord,int>>{
            {{5,2},fpga::ELEMENT_FD},{{0,2},fpga::ELEMENT_LUT5},
            {{0,2},fpga::ELEMENT_LUT1}}) {
        fpga::Element e; e.name="transit_owner_slot"; e.type=decltype(e.type)(type);
        f.tile(at).tile_type->elements.push_back(e);
    }
    auto* driver=f.cell("guard_driver","FDRE",{{"Q",true}},{5,2});
    auto* sink=f.cell("guard_receiver","LUT6",{{"I0",false}},{0,2});
    auto* victim=f.connect(*driver,"Q",*sink,"I0");
    f.edge({6,0},{3,0},169,902,650,true);
    f.tile({5,2}).tile_type->pin_map.output_nodes[
        f.tile({5,2}).getNodeNum("FDRE","Q",0)].setBit(141);
    f.tile({0,2}).tile_type->pin_map.nodes[{"LUT6","I0",3}].setBit(87);
    Route route{SourceInputFixture::pin({5,2},141,true),
        f.edge({5,2},{3,0},141,1300,650,true),
        f.edge({3,0},{0,2},650,1301,1400),
        f.terminal({0,2},1400,87),SourceInputFixture::pin({0,2},87,false)};
    for (auto& cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
    require(pnr::commitPreparedRoute(route),"cannot install real CE transit obstruction");
    sink->wires.push_back(std::move(route));
    auto id=fpga::attachNetRoute(*victim,*sink,0,driver,sink,"Q","I0",victim->name);
    fpga::registerNetRouteTiles(*victim,sink->wires.front(),id);
    f.auditAll();
    auto index=router.backwardDockingIndex({2,0},16);
    NodeMask pins; pins.setBit(63);
    auto trace=[&] {
        return pnr::traceBackwardToTakeoff(f.tile({2,0}),pins,{6,0},index,
            [&](fpga::Tile& tile,int src,pnr::BackwardTakeoffChoice& choice) {
                return router.routeInputSourceTry(*f.control,"O",tile,src,choice);
            });
    };
    require(!trace().success && fpga::isRouteComplete(sink->wires.front()),
        "CE trace crossed a foreign owned transit");
    require(fpga::unrouteNet(*victim),"cannot remove the genuine transit obstruction");
    f.auditAll();
    const auto opened=trace();
    require(opened.success && opened.source_tile == &f.tile({6,0}),
        "removing the transit did not expose the real CE source");
    f.auditAll();
}

void source_boundary_growing_input_regression() {
    run_source_boundary_restarts(false,false,true);
}
