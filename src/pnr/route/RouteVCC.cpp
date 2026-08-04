#include "RouteVCC.h"

#include "Design.h"
#include "Device.h"
#include "Tech.h"
#include "Tile.h"

#include <algorithm>
#include <cstdlib>
#include <format>
#include <limits>

namespace {

constexpr size_t FULL_NAME_LIMIT = std::numeric_limits<size_t>::max();

struct ConstantSink
{
    rtl::Inst* inst = nullptr;
    std::string port;
};

void collectConstantSinks(rtl::Inst& inst, rtl::Conn* constant,
                          std::vector<ConstantSink>& sinks)
{
    for (rtl::Conn& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        if (conn.follow() == constant) {
            sinks.push_back(ConstantSink{&inst, conn.port_ref->makeName()});
        }
    }
    for (rtl::Inst& child : inst.insts) {
        collectConstantSinks(child, constant, sinks);
    }
}

rtl::Module* topModule(technology::Tech& tech)
{
    return tech.design.top.cell_ref.peer
        ? tech.design.top.cell_ref->module_ref.peer : nullptr;
}

int nextGeneratedDesignator(const rtl::Module& module)
{
    int next = 100000000;
    for (const rtl::Net& net : module.nets) {
        for (int designator : net.designators) {
            next = std::max(next, designator + 1);
        }
    }
    return next;
}

rtl::Inst* existingSource(rtl::Inst& root, bool one)
{
    for (rtl::Inst& child : root.insts) {
        if (child.cell_ref.peer
            && child.cell_ref->attributes.contains("scalepnr_constant")
            && child.cell_ref->attributes.at("scalepnr_constant") == "1"
            && child.cell_ref->attributes.contains("scalepnr_constant_role")
            && child.cell_ref->attributes.at("scalepnr_constant_role")
                == "source"
            && (child.cell_ref->attributes.contains("scalepnr_constant_value")
                    ? child.cell_ref->attributes.at("scalepnr_constant_value") == (one ? "1" : "0")
                    : one)) {
            return &child;
        }
        if (rtl::Inst* nested = existingSource(child, one)) {
            return nested;
        }
    }
    return nullptr;
}

Referable<rtl::Cell>* makeSourceCell(rtl::Inst& model, int designator, bool one)
{
    auto* cell = new Referable<rtl::Cell>();
    cell->name = one ? "$scalepnr_constant_cell$1" : "$scalepnr_constant_cell$0";
    cell->type = "LUT2";
    cell->attributes["scalepnr_constant"] = "1";
    cell->attributes["scalepnr_constant_role"] = "source";
    cell->attributes["scalepnr_constant_value"] = one ? "1" : "0";
    if (model.cell_ref.peer && model.cell_ref->module_ref.peer) {
        cell->module_ref.set(model.cell_ref->module_ref.peer);
    }

    rtl::Port output;
    output.name = "O";
    output.type = rtl::Port::PORT_OUT;
    output.index = 0;
    output.designator = designator;
    cell->ports.emplace_back(std::move(output));
    return cell;
}

rtl::Inst& makeSource(technology::Tech& tech, rtl::Inst& model, int designator, bool one)
{
    auto& source = tech.design.top.insts.emplace_back();
    source.cell_ref.set(makeSourceCell(model, designator, one));
    source.parent_ref.set(&tech.design.top);
    source.cnt_outputs = 1;
    source.pos = -1;
    source.depth = model.depth;
    source.height = 1;

    rtl::Conn& output = source.conns.emplace_back();
    output.port_ref.set(&source.cell_ref->ports.front());
    output.inst_ref.set(static_cast<Referable<rtl::Inst>*>(&source));
    return source;
}

rtl::Net* existingNet(rtl::Module& module, const std::string& name)
{
    for (rtl::Net& net : module.nets) {
        if (net.name == name) {
            return &net;
        }
    }
    return nullptr;
}

rtl::Net& makeNet(rtl::Module& module, int designator, const std::string& name)
{
    rtl::Net& net = module.nets.emplace_back();
    net.name = name;
    net.designators.push_back(designator);
    return net;
}

bool hasDistributedSources(const fpga::Device& device, bool one)
{
    return std::any_of(device.tile_grid.begin(), device.tile_grid.end(),
        [one](const Referable<fpga::Tile>& tile) {
            return tile.cb_type
                && (one ? tile.cb_type->constant_one_nodes : tile.cb_type->constant_zero_nodes)
                    != NodeMask{};
        });
}

}

pnr::RouteVCC::RouteVCC(technology::Tech& tech, fpga::Device& device)
    : tech_(tech), device_(device)
{
}

pnr::RouteVCC::PreparedRoutes pnr::RouteVCC::prepareDesign()
{
    return prepareConstantDesign(tech_.design.VCC, true);
}

pnr::RouteVCC::PreparedRoutes pnr::RouteVCC::prepareGroundDesign()
{
    return prepareConstantDesign(tech_.design.GND, false);
}

pnr::RouteVCC::PreparedRoutes pnr::RouteVCC::prepareConstantDesign(rtl::Conn* constant, bool one)
{
    stats_ = {};
    PreparedRoutes prepared;
    std::vector<ConstantSink> sinks;
    collectConstantSinks(tech_.design.top, constant, sinks);
    stats_.logical_sinks = sinks.size();
    if (sinks.empty()) {
        return prepared;
    }

    PNR_ASSERT(hasDistributedSources(device_, one),
               "constant-{} loads exist but the device declares no distributed source nodes",
               one ? "one" : "zero");
    rtl::Module* module = topModule(tech_);
    PNR_ASSERT(module, "distributed constant routing requires a top module");

    int designator = nextGeneratedDesignator(*module);
    rtl::Inst* source = existingSource(tech_.design.top, one);
    if (!source) {
        source = &makeSource(tech_, *sinks.front().inst, designator, one);
    } else if (!source->conns.empty() && source->conns.front().port_ref.peer) {
        designator = source->conns.front().port_ref->designator;
    }
    const std::string net_name = one ? "VCC_NET" : "GND_NET";
    rtl::Net* net = existingNet(*module, net_name);
    if (!net) {
        net = &makeNet(*module, designator, net_name);
    }
    net->route_protected = true;
    net->distributed_source = true;
    net->distributed_one = one;

    // Endpoint-chain insertion is topology preparation, not route search.
    // Recollect afterward because a generated passthrough may become the sink.
    for (const ConstantSink& sink : sinks) {
        rtl::Inst* route_from = source;
        std::string from_port = "O";
        rtl::Inst* route_to = sink.inst;
        std::string to_port = sink.port;
        rtl::Net* route_net = net;
        bool changed = fpga::preparePassthroughRouteEndpoints(
            route_from, from_port, route_to, to_port, route_net, false);
        if (std::getenv("SCALEPNR_VCC_DEBUG")) {
            PNR_LOG1("ROUT",
                "constant endpoint prepare: original='{}'/'{}' changed={} physical='{}'/'{}'",
                sink.inst ? sink.inst->makeName(FULL_NAME_LIMIT) : std::string{},
                sink.port, changed,
                route_to ? route_to->makeName(FULL_NAME_LIMIT) : std::string{},
                to_port);
        }
        PNR_ASSERT(route_from == source && route_net == net,
                   "constant endpoint preparation changed distributed source ownership");
    }

    sinks.clear();
    collectConstantSinks(tech_.design.top, constant, sinks);
    prepared.source = source;
    prepared.net = net;
    prepared.sinks.reserve(sinks.size());
    for (const ConstantSink& sink : sinks) {
        if (!sink.inst) {
            continue;
        }
        prepared.sinks.push_back(Sink{
            sink.inst,
            sink.port,
            std::format("{}.{}.{}", net_name,
                        sink.inst->makeName(FULL_NAME_LIMIT), sink.port)});
        if (std::getenv("SCALEPNR_VCC_DEBUG")) {
            PNR_LOG1("ROUT", "constant endpoint task: sink='{}' type='{}' port='{}'",
                sink.inst->makeName(FULL_NAME_LIMIT),
                sink.inst->cell_ref.peer ? sink.inst->cell_ref->type : std::string{},
                sink.port);
        }
    }
    stats_.prepared_sinks = prepared.sinks.size();
    return prepared;
}
