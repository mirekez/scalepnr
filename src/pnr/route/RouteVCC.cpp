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

rtl::Inst* existingSource(rtl::Inst& root)
{
    for (rtl::Inst& child : root.insts) {
        if (child.cell_ref.peer
            && child.cell_ref->attributes.contains("scalepnr_constant")
            && child.cell_ref->attributes.at("scalepnr_constant") == "1"
            && child.cell_ref->attributes.contains("scalepnr_constant_role")
            && child.cell_ref->attributes.at("scalepnr_constant_role")
                == "source") {
            return &child;
        }
        if (rtl::Inst* nested = existingSource(child)) {
            return nested;
        }
    }
    return nullptr;
}

Referable<rtl::Cell>* makeSourceCell(rtl::Inst& model, int designator)
{
    auto* cell = new Referable<rtl::Cell>();
    cell->name = "$scalepnr_constant_cell$1";
    cell->type = "LUT2";
    cell->attributes["scalepnr_constant"] = "1";
    cell->attributes["scalepnr_constant_role"] = "source";
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

rtl::Inst& makeSource(technology::Tech& tech, rtl::Inst& model, int designator)
{
    auto& source = tech.design.top.insts.emplace_back();
    source.cell_ref.set(makeSourceCell(model, designator));
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

rtl::Net* existingNet(rtl::Module& module)
{
    for (rtl::Net& net : module.nets) {
        if (net.name == "VCC_NET") {
            return &net;
        }
    }
    return nullptr;
}

rtl::Net& makeNet(rtl::Module& module, int designator)
{
    rtl::Net& net = module.nets.emplace_back();
    net.name = "VCC_NET";
    net.designators.push_back(designator);
    return net;
}

bool hasDistributedSources(const fpga::Device& device)
{
    return std::any_of(device.tile_grid.begin(), device.tile_grid.end(),
        [](const Referable<fpga::Tile>& tile) {
            return tile.cb_type
                && tile.cb_type->constant_one_nodes != NodeMask{};
        });
}

}

pnr::RouteVCC::RouteVCC(technology::Tech& tech, fpga::Device& device)
    : tech_(tech), device_(device)
{
}

pnr::RouteVCC::PreparedRoutes pnr::RouteVCC::prepareDesign()
{
    stats_ = {};
    PreparedRoutes prepared;
    std::vector<ConstantSink> sinks;
    collectConstantSinks(tech_.design.top, tech_.design.VCC, sinks);
    stats_.logical_sinks = sinks.size();
    if (sinks.empty()) {
        return prepared;
    }

    PNR_ASSERT(hasDistributedSources(device_),
               "constant-one loads exist but the device declares no distributed source nodes");
    rtl::Module* module = topModule(tech_);
    PNR_ASSERT(module, "constant-one routing requires a top module");

    int designator = nextGeneratedDesignator(*module);
    rtl::Inst* source = existingSource(tech_.design.top);
    if (!source) {
        source = &makeSource(tech_, *sinks.front().inst, designator);
    } else if (!source->conns.empty() && source->conns.front().port_ref.peer) {
        designator = source->conns.front().port_ref->designator;
    }
    rtl::Net* net = existingNet(*module);
    if (!net) {
        net = &makeNet(*module, designator);
    }
    net->route_protected = true;
    net->distributed_source = true;

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
    collectConstantSinks(tech_.design.top, tech_.design.VCC, sinks);
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
            std::format("VCC_NET.{}.{}",
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
