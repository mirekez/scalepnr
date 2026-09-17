#include "Tech.h"
#include "Device.h"
#include "RouteClocks.h"
#include "tcl_pnr.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
void require(bool value, const std::string& message)
{
    if (!value) throw std::runtime_error(message);
}
void near(double actual, double expected, const std::string& message)
{
    require(std::abs(actual - expected) < 1e-8,
        message + ": " + std::to_string(actual) + " != " + std::to_string(expected));
}

struct Fixture {
    technology::Tech& tech = technology::Tech::current();
    Referable<rtl::Module> top, primitive;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    Tcl_Interp* interp = Tcl_CreateInterp();
    fpga::TileType tile_type{"MULTI_CLOCK_TILE", 1, 0};
    Referable<rtl::Inst> *fast_source, *slow_source, *logic, *fast_sink, *slow_sink, *data_clock;
    Referable<rtl::Inst> *fast_buffer, *slow_buffer;
    int next_net = 1;

    Fixture() {
        tech.clocked_ports = {{"FD", "C"}, {"ZZ_OTHER_REG", "D"}};
        tech.buffers_ports = {{"CLOCK_BUFFER", "O"}, {"OUT_BUFFER", "O"}};
        tech.comb_delays.map = {{"LUT2", {2, {0.2, 0.3}}}};
        top.name = "top";
        top.is_blackbox = false;
        primitive.name = "primitives";
        primitive.is_blackbox = true;
        primitive.parent_ref.set(&top);
        tech.design.top_cell.name = "top";
        tech.design.top_cell.module_ref.set(&top);
        tech.design.top.cell_ref.set(&tech.design.top_cell);
        tech.design.top.depth = 0;
        tech.design.top_cell.ports.reserve(64);
        tech.design.top.conns.reserve(64);
        for (int i = 0; i < 20; ++i) topPort("clk" + std::to_string(i), rtl::Port::PORT_IN);
        fast_buffer = make("fast_buffer", "CLOCK_BUFFER", {"I", "O"});
        slow_buffer = make("slow_buffer", "CLOCK_BUFFER", {"I", "O"});
        connect(&tech.design.top.conns[0], conn(fast_buffer, "I"));
        connect(&tech.design.top.conns[1], conn(slow_buffer, "I"));
        auto* second = make("fast_buffer_second", "CLOCK_BUFFER", {"I", "O"});
        connect(conn(fast_buffer, "O"), conn(second, "I"));
        fast_source = reg("fast_source", second);
        slow_source = reg("slow_source", slow_buffer);
        fast_sink = reg("fast_sink", second);
        slow_sink = reg("slow_sink", slow_buffer);
        data_clock = reg("clock_used_as_data", slow_buffer);
        logic = make("shared_logic", "LUT2", {"I0", "I1", "O"});
        connect(conn(fast_source, "Q"), conn(logic, "I0"));
        connect(conn(slow_source, "Q"), conn(logic, "I1"));
        connect(conn(logic, "O"), conn(fast_sink, "D"));
        connect(conn(logic, "O"), conn(slow_sink, "D"));
        connect(conn(fast_buffer, "O"), conn(data_clock, "D"));
        for (auto* sink : {fast_sink, slow_sink}) {
            auto* out = make(sink->cell_ref->name + "_out", "OUT_BUFFER", {"I", "O"});
            connect(conn(sink, "Q"), conn(out, "I"));
            connect(conn(out, "O"), topPort(out->cell_ref->name, rtl::Port::PORT_OUT));
        }
        Tcl_CreateObjCommand(interp, "create_clock", create_clock_cmd, nullptr, nullptr);
        Tcl_CreateObjCommand(interp, "get_ports", get_ports_cmd, nullptr, nullptr);
        Tcl_CreateObjCommand(interp, "get_clocks", get_clocks_cmd, nullptr, nullptr);
        Tcl_CreateObjCommand(interp, "set_clock_groups", set_clock_groups_cmd, nullptr, nullptr);
        fpga::Element fd;
        fd.name = "REG"; fd.type = fpga::ELEMENT_FD; fd.bitmap_pos = 0;
        fd.elements_to_left = fpga::ELEMENT_FD;
        tile_type.elements.push_back(fd);
        fpga::Element lut;
        lut.name = "LOGIC"; lut.type = fpga::ELEMENT_LUT5; lut.bitmap_pos = 0;
        lut.elements_to_left = fpga::ELEMENT_LUT5;
        tile_type.elements.push_back(lut);
        auto& device = fpga::Device::current();
        device.size_width = device.size_height = 12;
        device.grid_spec.size = {12, 12};
        device.tile_grid.resize(144);
        for (int i = 0; i < 144; ++i) {
            auto& tile = device.tile_grid[i];
            tile.coord = {i % 12, i / 12}; tile.cb_coord = tile.coord;
            tile.tile_type = &tile_type;
        }
    }
    ~Fixture() { Tcl_DeleteInterp(interp); }

    Referable<rtl::Conn>* topPort(std::string name, decltype(rtl::Port::type) type) {
        auto& port = tech.design.top_cell.ports.emplace_back();
        port.name = name; port.type = type;
        auto& connection = tech.design.top.conns.emplace_back();
        connection.port_ref.set(&port); connection.inst_ref.set(&tech.design.top);
        return &connection;
    }
    Referable<rtl::Inst>* make(std::string name, std::string type, std::vector<std::string> ports) {
        auto cell = std::make_unique<Referable<rtl::Cell>>();
        cell->name = name; cell->type = type; cell->module_ref.set(&primitive);
        cell->ports.reserve(ports.size());
        int input = 0;
        for (auto& name : ports) {
            auto& port = cell->ports.emplace_back();
            port.name = name;
            port.type = (name == "Q" || name == "O") ? rtl::Port::PORT_OUT : rtl::Port::PORT_IN;
            port.index = port.type == rtl::Port::PORT_IN ? input++ : 0;
        }
        auto& inst = tech.design.top.insts.emplace_back();
        inst.cell_ref.set(cell.get()); inst.parent_ref.set(&tech.design.top);
        inst.cnt_inputs = input; inst.cnt_outputs = 1; inst.pos = -1;
        inst.outline = {0, 0, false};
        inst.conns.reserve(ports.size());
        for (auto& port : cell->ports) {
            auto& connection = inst.conns.emplace_back();
            connection.port_ref.set(&port); connection.inst_ref.set(&inst);
        }
        cells.push_back(std::move(cell));
        return &inst;
    }
    Referable<rtl::Inst>* reg(std::string name, Referable<rtl::Inst>* clock) {
        auto* result = make(name, "FD", {"D", "C", "Q"}); // D before C is intentional
        connect(conn(clock, "O"), conn(result, "C"));
        return result;
    }
    Referable<rtl::Conn>* conn(Referable<rtl::Inst>* inst, const std::string& name) {
        for (auto& c : inst->conns) if (c.port_ref->name == name) return &c;
        throw std::runtime_error("missing port " + name);
    }
    void connect(Referable<rtl::Conn>* out, Referable<rtl::Conn>* in) {
        if (out->port_ref->designator <= 0) {
            out->port_ref->designator = next_net++;
            auto& net = top.nets.emplace_back();
            net.name = "net_" + std::to_string(out->port_ref->designator);
            net.designators.push_back(out->port_ref->designator);
        }
        in->port_ref->designator = out->port_ref->designator;
        in->set(out);
    }
    std::string eval(const std::string& script, bool success = true) {
        int result = Tcl_Eval(interp, script.c_str());
        std::string message = Tcl_GetStringResult(interp);
        require((result == TCL_OK) == success, script + ": " + message);
        return message;
    }
    void clocks() {
        eval("create_clock -period 1.0 -name fast [get_ports clk0]");
        eval("create_clock [get_ports clk1] -name slow -period 1.5");
    }
    void place(rtl::Inst* inst, int x, int y) {
        auto& tile = fpga::Device::current().tile_grid[y * 12 + x];
        tile.assign(inst); inst->coord = {x, y};
    }
    clk::Timings::TimingInfo& info(rtl::Clock* clock, rtl::Inst* inst) {
        for (auto& info : tech.timings.clocked_inputs.at(clock))
            if (info.data_in->inst_ref.peer == inst) return info;
        throw std::runtime_error("missing endpoint");
    }
};

void tclTest(Fixture& f) {
    f.clocks();
    auto* fast = &f.tech.clocks.clocks_list.front();
    near(f.info(fast, f.fast_sink).path.max_setup_time, 0.8,
        "Tcl clock declaration left placement timing pressure uncalculated");
    require(f.eval("get_clocks") == "fast slow", "clock declaration order lost");
    require(f.eval("get_clocks {f* s*}") == "fast slow", "clock patterns lost");
    require(f.eval("get_clocks {}").empty(), "empty pattern matched clocks");
    for (auto script : {"create_clock -period nope clk2", "create_clock -period -1 clk2",
         "create_clock -period Inf clk2", "create_clock -period 0 clk2",
         "create_clock -period 1 {}", "create_clock -period 1 {clk2 clk3}",
         "create_clock -period 1 -name fast clk2", "create_clock -period 1 clk0",
         "create_clock -period 1 missing", "create_clock -period 1 -waveform {0 0.5} clk2",
         "create_clock -period 1 -period 2 clk2"}) f.eval(script, false);
    require(f.tech.clocks.clocks_list.size() == 2, "invalid Tcl changed clock storage");
    auto count = f.tech.timings.clocked_inputs.at(fast).size();
    for (int i = 2; i < 20; ++i) f.eval("create_clock -period 2 clk" + std::to_string(i));
    require(fast == &f.tech.clocks.clocks_list.front(), "clock address changed above 16 clocks");
    require(f.tech.timings.clocked_inputs.at(fast).size() == count, "adding clocks duplicated timing endpoints");
    f.eval("set_clock_groups -asynchronous -group fast -group missing", false);
    f.eval("set_clock_groups -asynchronous -group fast -group fast", false);
    require(f.tech.clocks.asynchronous_pairs.empty(), "invalid group command mutated constraints");
    f.eval("set_clock_groups -asynchronous -group [get_clocks fast] -group [get_clocks slow]");
    require(f.tech.clocks.asynchronous(*fast, f.tech.clocks.clocks_list[1]), "async group missing");
    near(f.info(fast, f.fast_sink).path.max_setup_time, 0.2,
        "Tcl clock groups left stale synchronous timing pressure");
    std::string type = "FD", data = "D", clock = "C";
    require(!f.tech.check_clocked(type, data) && f.tech.check_clocked(type, clock),
        "clock-pin lookup leaked into next type (also affects data routing)");
    auto* top_cell = f.tech.design.top.cell_ref.peer;
    f.tech.design.top.cell_ref.clear();
    f.eval("create_clock -period 1 missing", false);
    f.tech.design.top.cell_ref.set(top_cell);
}

void timingTest(Fixture& f) {
    f.clocks();
    auto* fast = &f.tech.clocks.clocks_list[0];
    auto* slow = &f.tech.clocks.clocks_list[1];
    require(f.tech.timings.clocked_inputs.at(fast).size() == 2, "clock-as-data entered fast domain");
    require(f.tech.timings.clocked_inputs.at(slow).size() == 3, "slow domain endpoints missing");
    auto* replica = f.reg("fast_replica", f.fast_buffer);
    f.connect(f.conn(f.logic, "O"), f.conn(replica, "D"));
    for (int rebuild = 0; rebuild < 3; ++rebuild) {
        f.tech.prepareTimingLists(); f.tech.timings.calculateTimings();
        // 1 ns and 1.5 ns rising-edge clocks have 0.5 ns minimum separation.
        near(f.info(fast, f.fast_sink).path.max_setup_time, 0.8, "fast multirate launch");
        near(f.info(slow, f.slow_sink).path.max_setup_time, 1.2, "slow multirate launch");
        auto& duplicate = f.info(fast, replica).path;
        require(duplicate.precalculated == &f.info(fast, f.fast_sink).path,
            "same-domain shared cone was rebuilt or shared with a different domain");
    }
    f.eval("set_clock_groups -asynchronous -group fast -group slow");
    f.tech.timings.calculateTimings();
    near(f.info(fast, f.fast_sink).path.max_setup_time, 0.2, "fast cone leaked async input");
    near(f.info(slow, f.slow_sink).path.max_setup_time, 0.3, "slow cone leaked async input");
    require(f.info(fast, f.fast_sink).path.sub_paths.size() == 1
        && f.info(slow, f.slow_sink).path.sub_paths.size() == 1, "async cone pruning failed");
    require(f.conn(f.slow_source, "Q")->getPeers().size() == 1, "async constraints removed physical net");
    auto* cdc = f.reg("direct_crossing", f.slow_buffer);
    f.connect(f.conn(f.fast_source, "Q"), f.conn(cdc, "D"));
    f.tech.prepareTimingLists();
    require(!f.info(slow, cdc).constrained, "pure asynchronous path remained setup-constrained");
    auto placed = f.tech.place.place_timing.analyze(f.tech.timings);
    for (auto& endpoint : placed.endpoint_details)
        require(endpoint.data_in->inst_ref.peer != cdc, "async endpoint entered DEFICITE/PROFICITE maps");
}

void estimateTest(Fixture& f) {
    f.clocks();
    f.tech.estimate.clocks = &f.tech.clocks;
    f.tech.estimate.estimateDesign(f.tech.design);
    for (auto* cell : {f.fast_source, f.fast_sink, f.slow_source, f.slow_sink}) {
        auto* clock = f.tech.clocks.findClock(f.conn(cell, "C"), f.tech.buffers_ports);
        require(cell->bunch_ref.peer && cell->bunch_ref->clk_ref.peer == clock,
            "Estimate lost domain for " + cell->makeName());
    }
    auto* bunch = f.fast_sink->bunch_ref.peer;
    require(!bunch->uplinks.empty(), "Estimate lost shared combinational tree");
    for (auto& link : bunch->uplinks)
        near(link.deficit, link.delay - 1.0, "D-before-C lost clock in uplink weight");
    require(f.fast_source->bunch_ref.peer != f.slow_source->bunch_ref.peer,
        "different domains aggregated into one bunch");
}

void placementTest(Fixture& f) {
    f.clocks();
    f.place(f.fast_source, 0, 0); f.place(f.slow_source, 11, 11);
    f.place(f.logic, 5, 5); f.place(f.fast_sink, 11, 0); f.place(f.slow_sink, 0, 11);
    auto& timing = f.tech.place.place_timing;
    timing.tech = &f.tech;
    auto full = timing.analyze(f.tech.timings);
    require(full.endpoints >= 2, "multi-clock placed endpoints missing");
    double fast_input = timing.estimateWireDelay(*f.conn(f.logic, "I0"), *f.conn(f.fast_source, "Q"));
    double slow_input = timing.estimateWireDelay(*f.conn(f.logic, "I1"), *f.conn(f.slow_source, "Q"));
    for (auto& endpoint : full.endpoint_details) {
        if (endpoint.data_in->inst_ref.peer == f.fast_sink)
            near(endpoint.arrival_ns, std::max(fast_input + 0.2, slow_input + 0.3 + 0.5)
                + timing.estimateWireDelay(*f.conn(f.fast_sink, "D"), *f.conn(f.logic, "O")), "placed fast launch edge");
        if (endpoint.data_in->inst_ref.peer == f.slow_sink)
            near(endpoint.arrival_ns, std::max(fast_input + 0.2 + 1.0, slow_input + 0.3)
                + timing.estimateWireDelay(*f.conn(f.slow_sink, "D"), *f.conn(f.logic, "O")), "placed slow launch edge");
    }
    auto expected = [&](const pnr::PlaceTimingAnalysis& analysis) {
        auto reference = timing.analyze(f.tech.timings);
        require(reference.endpoint_details.size() == analysis.endpoint_details.size(), "endpoint count changed");
        for (size_t i = 0; i < reference.endpoint_details.size(); ++i)
            near(analysis.endpoint_details[i].slack_ns, reference.endpoint_details[i].slack_ns,
                 "local/full multi-clock timing mismatch");
    };
    {
        pnr::PlaceTimingLocal local(timing, full);
        f.logic->coord = {6, 4}; local.updateForward({f.logic}); expected(full);
    }
    auto prepared_state = full;
    std::vector<pnr::PlaceTimingEndpoint*> endpoints;
    for (auto& endpoint : prepared_state.endpoint_details) endpoints.push_back(&endpoint);
    pnr::PlaceTimingPrepared prepared(timing);
    prepared.evaluate(endpoints); expected(prepared_state);
    f.logic->coord = {7, 3}; prepared.evaluate(endpoints); expected(prepared_state);
    pnr::PlaceTimingIncremental incremental(timing, full);
    incremental.updateForward({f.logic}); expected(full);
    f.eval("set_clock_groups -asynchronous -group fast -group slow");
    f.tech.timings.calculateTimings();
    timing.preparePlacementGuide(f.tech.timings);
    require(timing.placementNetWeight(*f.fast_source, *f.logic)
        > timing.placementNetWeight(*f.slow_source, *f.logic), "Outline/packing guide ignores clock period");
    auto async = timing.analyze(f.tech.timings);
    for (auto& endpoint : async.endpoint_details)
        if (endpoint.data_in->inst_ref.peer == f.fast_sink)
            for (auto& edge : endpoint.critical_edges)
                require(edge.driver != f.slow_source, "async launch leaked into movement forces");
}

void routingTest(Fixture& f, bool conflict) {
    f.clocks();
    f.eval("create_clock -name unused -period 2 clk2");
    auto* branch = f.make("branch_buffer", "CLOCK_BUFFER", {"I", "O"});
    f.connect(f.conn(f.fast_buffer, "O"), f.conn(branch, "I"));
    auto* branch_sink = f.reg("branch_sink", branch);
    auto& device = fpga::Device::current();
    device.cb_types.emplace_back();
    auto& cb = device.cb_types.back();
    cb.name = "CLOCK_MESH"; cb.type_id = cb.base_type_id = 0;
    for (int node = 0; node < 3; ++node)
        cb.rememberNodeName(fpga::CB_NODE_LOCAL, node, "LOCAL_" + std::to_string(node));
    // A tiny fully connected dedicated fabric. Independent sources share
    // possible destinations but may never acquire another clock's sink node.
    for (int y = -11; y <= 11; ++y) for (int x = -11; x <= 11; ++x)
        for (int sink : {0, 2})
            cb.local_by_local[1].push_back(fpga::CBType::ResolvedLocal{
                {x, y}, 0, fpga::CB_NODE_LOCAL, NodeMask{0, 1} << sink});
    fpga::SiteModel site;
    site.name = "CLOCK_SITE"; site.type = "CLOCK_BUFFER"; site.pos = 0;
    for (auto [name, node] : {std::pair{"I", 2}, {"O", 1}, {"CLK", 0}}) {
        auto direction = node == 1 ? fpga::TILE_PIN_OUTPUT : fpga::TILE_PIN_INPUT;
        auto& pin = site.pins.emplace_back();
        pin.port = name; pin.site_pos = 0;
        pin.direction = node == 1 ? fpga::Pin::PIN_OUTPUT : fpga::Pin::PIN_INPUT;
        auto& map = f.tile_type.pin_map;
        map.rememberResourcePinName(direction, node, name);
        (node == 1 ? map.output_nodes : map.input_nodes)[node] = NodeMask{0, 1} << node;
    }
    f.tile_type.sites.push_back(site);
    for (auto& tile : device.tile_grid) { tile.cb_type = &cb; tile.cb.type = &cb; }
    int index = 0;
    for (auto& inst : f.tech.design.top.insts) {
        f.place(&inst, index % 12, index / 12); inst.pos = 0; ++index;
    }
    if (conflict) {
        f.slow_sink->tile.clear();
        f.place(f.slow_sink, f.fast_sink->coord.x, f.fast_sink->coord.y);
    }
    pnr::RouteClocks router(f.tech, device);
    require(router.routeDesign(f.tech.clocks) != conflict,
        conflict ? "two clocks acquired the same physical clock pin" : "multi-clock tree routing failed");
    require(router.stats().clocks == 3 && router.stats().nets == 4,
        "branched clock buffer tree lost nets");
    require(router.stats().sinks == 8, "clock routing swallowed a data pin or lost a clock sink");
    if (!conflict) {
        require(router.stats().routed == 8 && router.stats().failed == 0,
            "multi-clock sinks missing routes");
        for (auto* sink : {f.fast_source, f.fast_sink, f.slow_source, f.slow_sink, f.data_clock, branch_sink})
            require(sink->wires.size() == 1, "clock routes duplicated or crossed domains");
        for (auto& net : f.top.nets) for (auto& binding : net.routes)
            require(binding.to_port == "C" || binding.to_port == "I",
                "clock-as-data route was incorrectly handled as a clock pin");
    }
}

void flowTest(Fixture& f) {
    f.clocks();
    f.eval("set_clock_groups -asynchronous -group fast -group slow");
    f.tech.timings.calculateTimings();
    auto& device = fpga::Device::current();
    device.cnt_luts = device.cnt_regs = 144;
    f.tech.openDesign();
    int anchor = 0;
    for (auto& inst : f.tech.design.top.insts) {
        if (f.tech.buffers_ports.contains(inst.cell_ref->type) || &inst == f.data_clock) {
            f.place(&inst, anchor++, 11);
            inst.outline.fixed = true;
            inst.outline.x = inst.coord.x / 1.2F;
            inst.outline.y = inst.coord.y / 1.2F;
            inst.pos = 0;
            if (inst.bunch_ref.peer) {
                inst.bunch_ref->fixed = true;
                inst.bunch_ref->x = inst.outline.x;
                inst.bunch_ref->y = inst.outline.y;
            }
        }
    }
    auto check = [&](const std::string& stage) {
        auto analysis = f.tech.place.place_timing.analyze(f.tech.timings);
        size_t fast = 0, slow = 0;
        for (auto& endpoint : analysis.endpoint_details) {
            if (endpoint.clock->name == "fast") { ++fast; near(endpoint.required_ns, 1.0, stage + " fast budget"); }
            if (endpoint.clock->name == "slow") { ++slow; near(endpoint.required_ns, 1.5, stage + " slow budget"); }
        }
        require(fast == 1 && slow == 2, stage + " lost/duplicated clock domains");
        for (auto* reg : {f.fast_source, f.fast_sink, f.slow_source, f.slow_sink})
            require(reg->bunch_ref->clk_ref.peer == f.tech.clocks.findClock(f.conn(reg, "C"), f.tech.buffers_ports),
                stage + " mixed clock domains in bunches");
        std::cout << "MULTI_CLOCK_STAGE " << stage << " WNS=" << analysis.worst_slack_ns << '\n';
    };
    f.tech.place.write_debug_images = false;
    f.tech.outline.optimizeOutline(f.tech.estimate.data_outs); check("Outline");
    f.tech.place.placeDesign(f.tech.estimate.data_outs); check("PlaceDesign");
    for (auto* inst : {f.fast_source, f.fast_sink, f.slow_source, f.slow_sink, f.logic})
        require(inst->tile.peer && inst->pos >= 0, "PlaceDesign left cell unpacked");
    f.tech.place.refineTiming(f.tech.timings); check("PlaceTiming");
    f.tech.sorting.config.maximum_passes = 2;
    f.tech.sorting.config.maximum_runtime_seconds = 2;
    auto sorting = f.tech.sorting.run(f.tech.timings); check("PlaceSorting");
    require(!sorting.timed_out, "tiny multi-clock Sorting timed out");
    f.tech.swapping.config.maximum_passes = 2;
    f.tech.swapping.config.maximum_runtime_seconds = 2;
    f.tech.swapping.config.recovery_runtime_reserve_seconds = 0;
    auto swapping = f.tech.swapping.run(f.tech.timings); check("PlaceSwapping");
    require(!swapping.timed_out, "tiny multi-clock Swapping timed out");
}

void packingTest(Fixture& f) {
    f.clocks();
    f.tile_type.elements.clear();
    for (int bit = 0; bit < 3; ++bit) {
        fpga::Element element;
        element.type = fpga::ELEMENT_FD; element.bitmap_pos = bit;
        element.elements_to_left = fpga::ELEMENT_FD;
        element.clock_group = bit < 2 ? 0 : 1;
        f.tile_type.elements.push_back(element);
    }
    auto& tile = fpga::Device::current().tile_grid.front();
    require(tile.tryAddAt(f.fast_source, 0) == 0, "first clock register did not pack");
    auto* other_buffer = f.reg("same_domain_other_buffer", f.fast_buffer);
    require(tile.tryAddAt(other_buffer, 4) < 0,
        "different physical clock nets were merged because their declared domain matched");
    {
        fpga::ElementPackingPreview preview(tile);
        require(preview.reserveAt(f.slow_source, 4) < 0,
            "preview admitted incompatible clocks in one group");
        require(preview.reserveAt(f.fast_sink, 4) == 4,
            "same-clock register could not share clock resource");
        require(preview.reserveAt(f.slow_source, 8) == 8,
            "independent clock group could not host second clock");
    }
    require(!f.fast_sink->tile.peer && !f.slow_source->tile.peer,
        "clock packing preview changed actual placement");
    require(tile.tryAddAt(f.slow_source, 4) < 0, "packing ignored shared clock conflict");
    require(tile.tryAddAt(f.fast_sink, 4) == 4, "same clock failed to pack");
    require(tile.tryAdd(f.slow_source) == 8, "automatic packing did not choose independent clock group");
    near(f.tech.clocks.clocks_list[0].period_ns, 1.0, "packing changed clock constraints");
    fpga::TileType derived;
    for (int i = 0; i < 2; ++i) {
        auto& site = derived.sites.emplace_back();
        site.name = "SITE_" + std::to_string(i); site.pos = i;
        for (auto name : {"A", "AQ", "CLK"}) {
            auto& pin = site.pins.emplace_back(); pin.port = name;
            pin.direction = pin.port == "CLK" ? fpga::Pin::PIN_INPUT : fpga::Pin::PIN_OUTPUT;
        }
    }
    derived.rebuildElementsFromSites();
    size_t registers = 0;
    for (auto& element : derived.elements) if (element.type == fpga::ELEMENT_FD) {
        ++registers;
        require(element.clock_group == element.bitmap_pos / 8,
            "site-derived registers lost their shared clock group");
    }
    require(registers == 4, "site model did not generate both independent register groups");
}

void ambiguousClockTest(Fixture& f) {
    auto* dual = f.make("dual_clock", "DUAL_REG", {"D", "C0", "C1", "Q"});
    f.tech.clocked_ports.emplace("DUAL_REG", "C0");
    f.tech.clocked_ports.emplace("DUAL_REG", "C1");
    f.connect(f.conn(f.fast_buffer, "O"), f.conn(dual, "C0"));
    f.connect(f.conn(f.slow_buffer, "O"), f.conn(dual, "C1"));
    f.eval("create_clock -name fast -period 1 clk0");
    auto* clock = &f.tech.clocks.clocks_list[0];
    auto* old_path = &f.tech.timings.clocked_inputs.at(clock).front().path;
    auto message = f.eval("create_clock -name slow -period 1.5 clk1", false);
    require(message.find("ambiguous capture clocks") != std::string::npos,
        "unsupported dual-clock primitive silently used wrong clock");
    require(f.tech.clocks.clocks_list.size() == 1
        && old_path == &f.tech.timings.clocked_inputs.at(clock).front().path,
        "failed clock declaration damaged existing timing forest");
}

void repairTest(Fixture& f, bool swapping) {
    f.clocks();
    f.tech.clocks.clocks_list[0].period_ns = 0.1;
    f.conn(f.fast_sink, "D")->clear();
    f.conn(f.slow_sink, "D")->clear();
    f.connect(f.conn(f.fast_source, "Q"), f.conn(f.fast_sink, "D"));
    f.connect(f.conn(f.slow_source, "Q"), f.conn(f.slow_sink, "D"));
    f.eval("set_clock_groups -asynchronous -group fast -group slow");
    std::list<Referable<pnr::RegBunch>> bunches;
    std::vector<rtl::Inst*> cells{f.fast_source, f.fast_sink, f.slow_source, f.slow_sink};
    int positions[] = {0, 11, 5, 6};
    for (size_t i = 0; i < cells.size(); ++i) {
        auto* cell = cells[i];
        auto& bunch = bunches.emplace_back();
        bunch.reg = cell; bunch.size = bunch.size_regs = bunch.size_regs_own = 1;
        bunch.clk_ref.set(&f.tech.clocks.clocks_list[i < 2 ? 0 : 1]);
        cell->bunch_ref.set(&bunch);
        auto& tile = fpga::Device::current().tile_grid[positions[i]];
        require(tile.tryAddAt(cell, 0) == 0, "repair fixture failed to pack");
    }
    f.fast_source->outline.fixed = true;
    f.slow_sink->outline.fixed = true;
    auto& timing = f.tech.place.place_timing;
    auto before = timing.analyze(f.tech.timings);
    require(before.worst_slack_ns < -0.2, "repair fixture was not timing-critical");
    if (swapping) {
        f.tech.swapping.config.maximum_passes = 2;
        f.tech.swapping.config.maximum_runtime_seconds = 2;
        f.tech.swapping.config.recovery_runtime_reserve_seconds = 0;
        auto result = f.tech.swapping.run(f.tech.timings, cells);
        require(result.accepted_swaps > 0 && !result.timed_out,
            "multi-clock Swapping failed to use a slower-domain challenger");
    } else {
        f.tech.sorting.config.maximum_passes = 2;
        f.tech.sorting.config.maximum_runtime_seconds = 2;
        auto result = f.tech.sorting.run(f.tech.timings, cells);
        require(result.accepted_moves > 0 && !result.timed_out,
            "multi-clock Sorting did not shift the critical domain");
    }
    auto after = timing.analyze(f.tech.timings);
    require(after.worst_slack_ns > before.worst_slack_ns + 0.02,
        "multi-clock timing repair did not improve WNS");
    for (auto& endpoint : after.endpoint_details)
        if (endpoint.clock->name == "slow")
            require(endpoint.slack_ns >= 0, "repair broke the slower clock domain");
    for (size_t i = 0; i < cells.size(); ++i)
        require(cells[i]->bunch_ref->clk_ref.peer == &f.tech.clocks.clocks_list[i < 2 ? 0 : 1],
            "repair reassigned a clock domain");
}

void dataRoutingTest(Fixture& f) {
    f.clocks();
    auto* cdc = f.reg("cdc_sink", f.slow_buffer);
    f.connect(f.conn(f.fast_source, "Q"), f.conn(cdc, "D"));
    f.eval("set_clock_groups -asynchronous -group fast -group slow");
    auto& device = fpga::Device::current();
    device.cb_types.emplace_back();
    auto& cb = device.cb_types.back();
    cb.name = "DATA_MESH"; cb.type_id = cb.base_type_id = 0;
    auto bit = [](int n) { return NodeMask{0, 1} << n; };
    for (int local : {1, 16, 17, 18, 31})
        cb.rememberNodeName(fpga::CB_NODE_LOCAL, local, "LOCAL_" + std::to_string(local));
    auto arc = [&](fpga::CBNodeNameType from_type, int from, fpga::CBNodeNameType to_type, int to) {
        cb.rememberConnName(from_type, from, to_type, to,
            *cb.nodeName(from_type, from), *cb.nodeName(to_type, to));
    };
    fpga::Coord directions[] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    for (int node = 0; node < 12; ++node) {
        cb.rememberNodeName(fpga::CB_NODE_SRC, node, "SRC_" + std::to_string(node));
        cb.rememberNodeName(fpga::CB_NODE_DST, node, "DST_" + std::to_string(node));
        fpga::CBType::ResolvedJump jump;
        jump.delta = directions[node / 3]; jump.target_cb_type_id = 0; jump.dsts.jump = bit(node);
        cb.dst_by_src[node].push_back(jump);
        arc(fpga::CB_NODE_SRC, node, fpga::CB_NODE_DST, node);
        for (int local : {1, 16}) {
            cb.local_src[local].jump |= bit(node);
            cb.local_output_nodes |= bit(local);
            arc(fpga::CB_NODE_LOCAL, local, fpga::CB_NODE_SRC, node);
        }
        for (int local : {17, 18, 31}) {
            cb.dst_local[node].local |= bit(local);
            cb.local_input_nodes |= bit(local);
            arc(fpga::CB_NODE_DST, node, fpga::CB_NODE_LOCAL, local);
        }
        cb.valid_dst_nodes |= bit(node);
    }
    for (int dst = 0; dst < 12; ++dst) for (int src = 0; src < 12; ++src) {
        cb.dst_src[dst].jump |= bit(src);
        arc(fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, src);
    }
    cb.rebuildOutgoingSrcs(); cb.ensureDerivedMasks();
    for (auto& tile : device.tile_grid) { tile.cb_type = &cb; tile.cb.type = &cb; }
    device.rebuildIncomingDstMasks();
    std::vector<rtl::Inst*> cells{f.fast_source, f.slow_source, f.logic, f.fast_sink, f.slow_sink, cdc};
    fpga::Coord coords[] = {{0, 0}, {6, 6}, {3, 3}, {7, 0}, {0, 7}, {8, 5}};
    for (size_t i = 0; i < cells.size(); ++i) {
        auto& tile = device.tile_grid[coords[i].y * 12 + coords[i].x];
        require(tile.tryAddAt(cells[i], cells[i] == f.logic ? 3 : 0) >= 0, "data-routing fixture packing failed");
    }
    auto& router = f.tech.route;
    router.fpga_width = router.fpga_height = 12;
    router.travers_mark = rtl::Inst::genMark();
    router.iteration_limit = 32;
    router.collectRouteTasks(*f.fast_sink);
    router.collectRouteTasks(*f.slow_sink);
    router.collectRouteTasks(*cdc);
    require(router.route_todo.size() == 3 && router.fanout_route_todo.size() == 2,
        "multi-clock data collection lost data pins or async crossing");
    auto basic = std::move(router.route_todo);
    auto suffixes = std::move(router.fanout_route_todo);
    std::vector<pnr::RouteDesign::RouteTask> fanout, moving;
    for (auto& task : suffixes) {
        require(task.to_port != "C", "generic routing collected a clock pin");
        (task.to == cdc ? moving : fanout).push_back(task);
    }
    auto first = router.routeTaskBatch(pnr::RouteDesign::RouteTaskMode::Generic, basic);
    require(first.completed == 3, "Basic failed on two-clock data paths");
    auto second = router.routeTaskBatch(pnr::RouteDesign::RouteTaskMode::Fanout, fanout);
    require(second.completed == 1, "Fanouts failed on shared mixed-domain logic");
    router.moving_stage = true; router.route_deadends_enabled = false;
    auto third = router.routeTaskBatch(pnr::RouteDesign::RouteTaskMode::Moving, moving);
    require(third.completed == 1, "Moving failed to route an asynchronous crossing");
    size_t bindings = 0;
    for (auto& net : f.top.nets) for (auto& binding : net.routes) {
        ++bindings;
        require(binding.to_port != "C", "data routing claimed a clock input");
    }
    require(bindings == 5, "data routing lost a physical connection across clock domains");
}
}

int main(int argc, char** argv) {
    try {
        Tcl_FindExecutable(argv[0]);
        Fixture fixture;
        std::string mode = argc > 1 ? argv[1] : "timing";
        if (mode == "tcl") tclTest(fixture);
        else if (mode == "timing") timingTest(fixture);
        else if (mode == "estimate") estimateTest(fixture);
        else if (mode == "placement") placementTest(fixture);
        else if (mode == "routing") routingTest(fixture, false);
        else if (mode == "routing_conflict") routingTest(fixture, true);
        else if (mode == "flow") flowTest(fixture);
        else if (mode == "packing") packingTest(fixture);
        else if (mode == "ambiguous_clock") ambiguousClockTest(fixture);
        else if (mode == "sorting") repairTest(fixture, false);
        else if (mode == "swapping") repairTest(fixture, true);
        else if (mode == "data_routing") dataRoutingTest(fixture);
        else throw std::runtime_error("unknown test mode");
        std::cout << "multi_clock " << mode << " passed\n";
    } catch (const std::exception& e) {
        std::cerr << "multi_clock: " << e.what() << '\n'; return 1;
    }
}
