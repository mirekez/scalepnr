#include "SourceInputFixture.h"
#include "RoutePassState.h"

// A real source move repeatedly proved its first input (about 269,000
// expansions), only to reject its second input from an independent negative
// proof. The next output/slot repeated that expensive work on the same tile.
// Keep that ordering and many exits, but use small, neutral-named topology.
// The new contract commits a candidate then recursively routes its inputs;
// this fixture must succeed without any speculative input proof.
void source_input_preflight_regression(bool connect_second) {
    using namespace routing_tests;
    setenv("SCALEPNR_SOURCE_TIMEOUT_TRACE_MS", "10000", 1);
    SourceInputFixture f(false, 0, 16);
    auto add_slot = [&](fpga::Coord at, fpga::ElementType kind) {
        fpga::Element e;
        e.name = "extra_slot";
        e.type = kind;
        f.tile(at).tile_type->elements.push_back(e);
    };
    add_slot({7,2}, fpga::ELEMENT_FD);
    for (fpga::Coord at : {fpga::Coord{6,2}, fpga::Coord{5,2}}) {
        add_slot(at, fpga::ELEMENT_LUT5);
        fpga::Element extension;
        extension.name = "extra_extension";
        extension.type = fpga::ELEMENT_LUT1;
        extension.bitmap_pos = 0;
        f.tile(at).tile_type->elements.push_back(extension);
    }
    auto *source = f.cell("moving_register", "FDRE",
                         {{"CE",false},{"D",false},{"Q",true}}, {7,2});
    auto *control = f.cell("first_driver", "LUT6", {{"O",true}}, {6,2});
    auto *data = f.cell("second_driver", "LUT6", {{"O",true}}, {5,2});
    auto *output = f.connect(*source, "Q", *f.sink, "I0");
    auto *first_input = f.connect(*control, "O", *source, "CE");
    auto *second_input = f.connect(*data, "O", *source, "D");
    for (auto &tile : f.device.tile_grid) {
        tile.tile_type->pin_map.nodes[{"FDRE","D",0}].setBit(88);
        tile.cb_type->local_input_nodes.setBit(88);
        SourceInputFixture::name(tile, fpga::CB_NODE_LOCAL, 88);
    }
    // The nearest candidate has 32 output choices and a valid first input,
    // but no path to its second input. The later candidate accepts all three.
    f.edge({6,2},{2,0},169,1500,1500,true);
    f.terminal({2,0},1500,63);
    // D has a free terminal and incoming fabric, so local availability alone
    // cannot reject it; that fabric is disconnected from its actual driver.
    f.edge({3,0},{2,0},1503,1503,1503);
    f.terminal({2,0},1503,88);
    f.edge({6,2},{1,0},169,1501,1501,true);
    f.terminal({1,0},1501,63);
    f.edge({5,2},{1,0},169,1502,1502,true);
    f.terminal({1,0},1502,88);
    if (connect_second) {
        f.edge({5,2},{2,0},169,1504,1504,true);
        f.terminal({2,0},1504,88);
    }
    for (auto &cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
    f.auditAll();

    auto &router = f.tech.route;
    router.moving_stage = router.moving_sources_stage = true;
    router.route_preemption_enabled = router.route_deadends_enabled = false;
    router.iteration_limit = router.route_suffix_depth_limit = 200;
    router.route_stage_deadline_enabled = true;
    router.route_stage_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    pnr::RouteDesign::RouteTask task{source,f.sink,output,"Q","I0","output_signal"};
    task.endpoints_prepared = true;
    std::ostringstream log;
    bool complete = false;
    std::string reason;
    {
        struct Capture {
            std::streambuf *old;
            explicit Capture(std::ostream &out) : old(std::cout.rdbuf(out.rdbuf())) {}
            ~Capture() { std::cout.rdbuf(old); }
        } capture(log);
        complete = router.moveUnfinishedSource(task,nullptr,&reason);
    }
    // Rejection must not discard valid relocation or corrupt existing owners.
    require(complete && source->tile.peer == &f.tile(connect_second ? fpga::Coord{2,0} : fpga::Coord{1,0}),
            "preflight lost the later compatible source placement: " + reason);
    // A placement-only success is insufficient: every incident route must
    // have a published complete binding with all leases owned consistently.
    for (auto *net : {output, first_input, second_input}) {
        bool routed = false;
        for (const auto &binding : net->routes)
            if (binding.owner && binding.route_index < binding.owner->wires.size())
                routed |= fpga::isRouteComplete(binding.owner->wires[binding.route_index]);
        require(routed, "source candidate committed without a complete incident route");
    }
    f.auditAll();
    const std::string text = log.str();
    size_t private_bad_proofs = 0;
    std::istringstream lines(text);
    for (std::string line; std::getline(lines,line);) {
        if (line.find("SOURCE_CANDIDATE_INPUT_VIEW") != std::string::npos &&
            line.find("resource=(2, 0)") != std::string::npos)
            ++private_bad_proofs;
    }
    std::cout << "source input preflight second_connected=" << connect_second
              << " first-placement private proofs="
              << private_bad_proofs << '\n';
    // The reachable early placement must be tried even when a later input
    // rejects it. Only that actual recursive failure may select the next one.
    const auto& tried=router.move_tried_placements.at(reinterpret_cast<uintptr_t>(source));
    const auto early=std::find(tried.begin(),tried.end(),pnr::movingPlacementKey(2,0,0));
    require(early!=tried.end(),"recursive move skipped its reachable first placement");
    if(!connect_second) {
        const auto later=std::find(tried.begin(),tried.end(),pnr::movingPlacementKey(1,0,0));
        require(later!=tried.end() && early<later,
                "failed second input did not lead to the next parent placement");
    }
    require(private_bad_proofs==0 && text.find("INPUT_PROOF_END")==std::string::npos,
            "recursive input routing restored a removed speculative proof");
}
