#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Inst.h"
#include "Wire.h"

#include <array>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
struct Point { int x, y, kind, node; };
void check(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}

// Captured free-node witness from a real Moving-source failure. Keep every
// numeric arc and coordinate, but give the synthetic boxes/nodes abstract
// names. No placement or competitor is needed to reproduce the rejection:
// all 99 nodes were free, and the 45-hop route must cross one box three times
// through DIFFERENT nodes. These are port changes, not a physical node cycle.
constexpr std::array<Point, 99> witness{{
    {70, 1, 0, 143},
    {70, 1, 3, 978},
    {75, 4, 4, 817},
    {75, 4, 3, 1026},
    {81, 4, 4, 1025},
    {81, 4, 3, 817},
    {85, 8, 4, 817},
    {85, 8, 3, 97},
    {85, 14, 4, 97},
    {85, 14, 3, 817},
    {91, 18, 4, 817},
    {91, 18, 3, 97},
    {91, 24, 4, 97},
    {91, 24, 3, 97},
    {91, 31, 4, 97},
    {91, 31, 3, 97},
    {91, 37, 4, 97},
    {91, 37, 3, 3377},
    {85, 41, 4, 3377},
    {85, 41, 3, 3078},
    {75, 41, 4, 3079},
    {75, 41, 3, 3377},
    {70, 45, 4, 3377},
    {70, 45, 3, 3078},
    {62, 45, 4, 3079},
    {62, 45, 3, 3078},
    {53, 45, 4, 3079},
    {53, 45, 3, 3078},
    {44, 45, 4, 3079},
    {44, 45, 3, 162},
    {44, 39, 4, 162},
    {44, 39, 3, 3078},
    {36, 39, 4, 3079},
    {36, 39, 3, 3078},
    {25, 39, 4, 3079},
    {25, 39, 3, 3078},
    {15, 39, 4, 3079},
    {15, 39, 3, 3538},
    {11, 35, 4, 3538},
    {11, 35, 3, 3538},
    {4, 31, 4, 3538},
    {4, 31, 3, 162},
    {4, 24, 4, 162},
    {4, 24, 3, 243},
    {4, 23, 4, 247},
    {4, 23, 3, 248},
    {4, 22, 4, 242},
    {4, 22, 3, 247},
    {4, 21, 4, 241},
    {4, 21, 3, 245},
    {4, 21, 4, 240},
    {4, 21, 3, 249},
    {4, 20, 4, 244},
    {4, 20, 3, 248},
    {4, 19, 4, 242},
    {4, 19, 3, 1026},
    {11, 19, 4, 1025},
    {11, 19, 3, 3842},
    {5, 19, 4, 3847},
    {5, 19, 3, 3843},
    {4, 19, 4, 3848},
    {4, 19, 3, 22},
    {4, 20, 4, 26},
    {4, 20, 0, 128},
    {4, 20, 3, 1031},
    {15, 20, 4, 1031},
    {15, 20, 3, 3840},
    {12, 19, 4, 3845},
    {12, 19, 0, 165},
    {12, 19, 0, 184},
    {12, 19, 3, 3078},
    {4, 19, 4, 1030},
    {4, 19, 3, 260},
    {5, 19, 4, 263},
    {5, 19, 3, 262},
    {11, 18, 4, 260},
    {11, 18, 0, 178},
    {11, 18, 0, 165},
    {11, 18, 3, 3078},
    {5, 18, 4, 1030},
    {5, 18, 3, 3843},
    {4, 18, 4, 3848},
    {4, 18, 3, 249},
    {4, 17, 4, 244},
    {4, 17, 3, 248},
    {4, 16, 4, 242},
    {4, 16, 3, 193},
    {4, 14, 4, 193},
    {4, 14, 3, 193},
    {4, 12, 4, 193},
    {4, 12, 3, 161},
    {4, 6, 4, 161},
    {4, 6, 3, 737},
    {5, 5, 4, 737},
    {5, 5, 3, 1027},
    {16, 5, 4, 1029},
    {16, 5, 3, 17},
    {16, 6, 4, 20},
    {16, 6, 0, 99},
}};
}

void moving_source_physical_tile_reentry() {
    auto& device = fpga::Device::current();
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width = 92; device.size_height = 46;
    device.grid_spec.size = {92, 46};
    device.tile_grid.resize(92 * 46);
    for (int y = 0; y < 46; ++y) for (int x = 0; x < 92; ++x)
        device.getTile(x,y)->coord = {x,y};
    std::map<std::pair<int,int>, size_t> boxes;
    for (const auto& point : witness)
        boxes.try_emplace({point.x, point.y}, boxes.size());
    device.cb_types.resize(boxes.size());
    for (const auto& [coord, id] : boxes) {
        auto* tile = device.getTile(coord.first, coord.second);
        auto& cb = device.cb_types[id];
        cb.type_id = cb.base_type_id = id;
        cb.name = "matrix_" + std::to_string(id);
        tile->cb_type = tile->cb.type = &cb;
    }
    for (const auto& point : witness)
        device.getTile(point.x, point.y)->cb_type->rememberNodeName(
            static_cast<fpga::CBNodeNameType>(point.kind), point.node,
            "node_" + std::to_string(point.kind) + "_" + std::to_string(point.node));
    size_t hops = 0;
    std::map<std::pair<int,int>, unsigned> arrivals;
    for (size_t i = 0; i < witness.size(); ++i) {
        const auto& point = witness[i];
        if (!i || point.x != witness[i-1].x || point.y != witness[i-1].y) {
            ++arrivals[{point.x, point.y}];
            hops += i != 0;
        }
        if (!i) continue;
        const auto& a = witness[i-1];
        auto* from = device.getTile(a.x, a.y);
        auto* to = device.getTile(point.x, point.y);
        auto& cb = *from->cb_type;
        if (a.kind == fpga::CB_NODE_SRC && point.kind == fpga::CB_NODE_DST) {
            NodeMask landing; landing.setBit(point.node);
            cb.dst_by_src[a.node].push_back({{point.x-a.x,point.y-a.y},
                static_cast<uint16_t>(to->cb_type->type_id), {landing}, {}, true});
            to->incoming_dst_nodes.setBit(point.node);
        } else {
            check(a.x == point.x && a.y == point.y, "unexpected cross-box internal arc");
            if (a.kind == fpga::CB_NODE_LOCAL && point.kind == fpga::CB_NODE_SRC)
                cb.local_src[a.node].jump.setBit(point.node);
            else if (a.kind == fpga::CB_NODE_DST && point.kind == fpga::CB_NODE_SRC)
                cb.dst_src[a.node].jump.setBit(point.node);
            else if (a.kind == fpga::CB_NODE_DST && point.kind == fpga::CB_NODE_LOCAL)
                cb.dst_local[a.node].local.setBit(point.node);
            else if (a.kind == fpga::CB_NODE_LOCAL && point.kind == fpga::CB_NODE_LOCAL)
                cb.local_local[a.node].local.setBit(point.node);
            else check(false, "unexpected captured internal arc");
        }
    }
    for (auto& cb : device.cb_types) cb.rebuildOutgoingSrcs();
    check(hops == 45 && arrivals[{4,19}] == 3, "captured reentry geometry changed");
    Referable<rtl::Net> net; net.name = "signal_501";
    Referable<rtl::Cell> driver_cell, sink_cell;
    driver_cell.name = "driver_502"; sink_cell.name = "sink_503";
    rtl::Inst driver, sink;
    driver.cell_ref.set(&driver_cell); sink.cell_ref.set(&sink_cell);
    rtl::NetRouteBinding endpoints;
    endpoints.from = &driver; endpoints.to = &sink;
    endpoints.from_port = "Q"; endpoints.to_port = "D"; endpoints.route_name = "route_504";
    fpga::Wire output, input;
    output.type = input.type = fpga::Wire::WIRE_TILE_PIN;
    output.from = {witness.front().x, witness.front().y}; output.local = witness.front().node;
    input.from = {witness.back().x, witness.back().y}; input.local = witness.back().node;
    output.pin_dir = fpga::TILE_PIN_OUTPUT; input.pin_dir = fpga::TILE_PIN_INPUT;
    fpga::RepairIncomingSources incoming = [&](fpga::Coord at, fpga::CBNodeNameType type,
                                               int node, const auto& visit) {
        for (size_t i = 1; i < witness.size(); ++i) {
            const auto& a = witness[i-1]; const auto& b = witness[i];
            if (a.kind == fpga::CB_NODE_SRC && b.kind == fpga::CB_NODE_DST &&
                b.x == at.x && b.y == at.y && b.kind == type && b.node == node)
                visit(fpga::Coord{a.x,a.y}, fpga::CB_NODE_SRC, a.node);
        }
    };
    std::ostringstream physical;
    check(fpga::auditUnroutedReachability(net, endpoints, output, input, physical, 512),
          "captured physical witness is not continuous: " + physical.str());
    auto audit = [&] {
        for (const auto& [coord,id] : boxes) {
            std::ostringstream details;
            const auto result = fpga::auditTileCongestion(
                *device.getTile(coord.first, coord.second), details, {&net});
            check(!result.orphan_leases && !result.missing_leases &&
                      !result.missing_registrations && !result.stale_registrations &&
                      !result.conflicting_driver_nodes,
                  "captured witness has invalid owners: " + details.str());
        }
    };
    audit();
    check(!fpga::routeUnboundNetFromSource(net, endpoints, output, input, {}, 512, 44, {}, incoming) &&
              net.routes.empty() && sink.wires.empty(),
          "insufficient hop budget committed a partial witness");
    audit();
    check(fpga::routeUnboundNetFromSource(net, endpoints, output, input, {}, 512, 200, {}, incoming),
          "free two-ended completion rejected a simple physical path with three box visits");
    check(net.routes.size() == 1 && sink.wires.size() == 1 &&
              fpga::isRouteComplete(sink.wires.front()), "captured witness was not completely installed");
    size_t installed_hops = 0;
    for (const auto& wire : sink.wires.front())
        if ((wire.type == fpga::Wire::WIRE_CROSSBAR || wire.type == fpga::Wire::WIRE_ROUTE_EDGE) &&
            (wire.from.x != wire.to.x || wire.from.y != wire.to.y)) ++installed_hops;
    check(installed_hops == hops, "completion changed the captured physical hop count");
    audit();
}
