#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Module.h"
#include "Wire.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
NodeMask bit(int node) { return NodeMask{0, 1} << node; }

// Run-4 CE failure, reduced to source CB, destination CB and attached resource
// Tile. Preserve the real terminal IDs and 14-busy/2-free split, but use only
// abstract node names. Every occupied node has an actual registered route.
struct ConstantFixture {
    static constexpr int root = 64, gate = 63, relay = 48;
    static constexpr int enable = 40, lut_input = 105, alternate = 90;
    static constexpr std::array<int, 16> arrivals{
        20, 24, 65, 193, 244, 246, 257, 261,
        545, 737, 1025, 3073, 3617, 3810, 3841, 3846};

    fpga::Device& device = fpga::Device::current();
    fpga::TileType resource_type{"resource_bank_842", 1, 0};
    fpga::CBType* cb = nullptr;
    Referable<rtl::Module> top, primitive;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;
    Referable<rtl::Inst>* old_lut = nullptr;
    Referable<rtl::Inst>* new_reg = nullptr;
    rtl::Net* old_net = nullptr;
    bool blocked;

    Referable<fpga::Tile>& source() { return device.tile_grid[0]; }
    Referable<fpga::Tile>& matrix() { return device.tile_grid[1]; }
    Referable<fpga::Tile>& resource() { return device.tile_grid[2]; }

    Referable<rtl::Inst>* cell(const std::string& name, const std::string& type,
                             std::initializer_list<std::pair<const char*, int>> ports)
    {
        auto c = std::make_unique<Referable<rtl::Cell>>();
        c->name = name; c->type = type; c->module_ref.set(&primitive);
        for (auto [name, direction] : ports) {
            auto& p = c->ports.emplace_back();
            p.name = name; p.type = static_cast<decltype(p.type)>(direction);
        }
        auto i = std::make_unique<Referable<rtl::Inst>>();
        i->cell_ref.set(c.get()); i->pos = -1;
        i->cnt_inputs = i->cnt_outputs = 0;
        for (auto& p : c->ports) {
            auto& conn = i->conns.emplace_back();
            conn.port_ref.set(&p); conn.inst_ref.set(i.get());
            i->cnt_inputs += p.type == rtl::Port::PORT_IN;
            i->cnt_outputs += p.type == rtl::Port::PORT_OUT;
        }
        auto* result = i.get();
        cells.push_back(std::move(c)); insts.push_back(std::move(i));
        return result;
    }

    Referable<rtl::Conn>* port(rtl::Inst& inst, const std::string& name)
    {
        for (auto& conn : inst.conns)
            if (conn.port_ref->name == name) return &conn;
        throw std::runtime_error("missing synthetic port");
    }

    rtl::Net& connect(rtl::Inst& driver, rtl::Inst& sink, const std::string& input)
    {
        int designator = static_cast<int>(top.nets.size()) + 1;
        auto* out = port(driver, "O"); auto* in = port(sink, input);
        out->port_ref->designator = in->port_ref->designator = designator;
        in->set(out);
        auto& net = top.nets.emplace_back();
        net.name = "signal_" + std::to_string(designator);
        net.designators.push_back(designator);
        return net;
    }

    void incoming(int dst, int src)
    {
        cb->rememberNodeName(fpga::CB_NODE_DST, dst, "arrival_" + std::to_string(dst));
        cb->rememberNodeName(fpga::CB_NODE_SRC, src, "departure_" + std::to_string(src));
        fpga::CBType::ResolvedJump jump{};
        jump.delta = {1, 0}; jump.target_cb_type_id = 0; jump.dsts.jump.setBit(dst);
        cb->dst_by_src[src].push_back(jump);
        matrix().incoming_dst_nodes.setBit(dst);
    }

    void route(rtl::Net& net, rtl::Inst& driver, rtl::Inst& sink,
               const std::string& input, int dst, int src, int local, bool via_gate)
    {
        int output = src - 700;
        cb->local_src[output].jump.setBit(src);
        cb->rememberNodeName(fpga::CB_NODE_LOCAL, output, "output_" + std::to_string(output));
        cb->rememberNodeName(fpga::CB_NODE_LOCAL, local, "input_" + std::to_string(local));
        if (!via_gate) cb->dst_local[dst].local.setBit(local);
        driver.tile.set(&source()); driver.coord = source().coord;
        if (!sink.tile.peer) { sink.tile.set(&resource()); sink.coord = resource().coord; }
        driver.outline.fixed = driver.pnr_completed = true;
        sink.outline.fixed = sink.pnr_completed = true;
        fpga::Wire out;
        out.type = fpga::Wire::WIRE_TILE_PIN;
        out.from = out.to = source().coord; out.local = output; out.pin_dir = 1;
        fpga::Wire hop;
        hop.from = source().coord; hop.to = matrix().coord;
        hop.pos = 0; hop.local = output; hop.jump = src; hop.dst = dst;
        fpga::Wire ground;
        ground.from = ground.to = matrix().coord; ground.pos = 1; ground.local = dst;
        if (via_gate) { ground.joint = relay; ground.joint2 = gate; }
        fpga::Wire pin;
        pin.type = fpga::Wire::WIRE_TILE_PIN;
        pin.from = pin.to = matrix().coord; pin.local = local; pin.pin_dir = 0;
        sink.wires.push_back({out, hop, ground, pin});
        source().cb.src.jump.setBit(src);
        matrix().cb.dst.jump.setBit(dst);
        matrix().cb.local.local.setBit(local);
        matrix().pin_state.leased_nodes.setBit(local);
        if (via_gate) { matrix().cb.joint.jump.setBit(gate); matrix().cb.joint.jump.setBit(relay); }
        net.pnr_preserved = true;
        auto b = fpga::attachNetRoute(net, sink, 0, &driver, &sink, "O", input, net.name);
        fpga::registerNetRouteTiles(net, sink.wires[0], b);
        require(fpga::isRouteComplete(sink.wires[0]), "fixture route is incomplete");
    }

    explicit ConstantFixture(bool blocked = true) : blocked(blocked)
    {
        device.tile_grid.clear(); device.cb_types.clear();
        device.cb_types.emplace_back(); cb = &device.cb_types.back();
        cb->name = "matrix_842"; cb->type_id = cb->base_type_id = 0;
        device.size_width = 3; device.size_height = 1; device.grid_spec.size = {3, 1};
        device.tile_grid.resize(3);
        for (int x = 0; x < 3; ++x) {
            auto& tile = device.tile_grid[x];
            tile.coord = tile.name = {x, 0}; tile.cb_coord = x == 2 ? fpga::Coord{1, 0} : tile.coord;
            tile.cb_type = cb; tile.cb.type = cb;
        }
        top.name = "constant_test_top"; top.nets.reserve(32);
        primitive.name = "primitive"; primitive.is_blackbox = true; primitive.parent_ref.set(&top);
        for (auto type : {fpga::ELEMENT_LUT5, fpga::ELEMENT_LUT1, fpga::ELEMENT_FD}) {
            fpga::Element e;
            e.name = "slot_" + std::to_string(type); e.type = type;
            e.bitmap_pos = 0; e.elements_to_left = static_cast<int>(type);
            resource_type.elements.push_back(e);
        }
        resource_type.pin_map.nodes[{"LUT6", "I4", 3}] = bit(lut_input);
        resource_type.pin_map.nodes[{"FDRE", "CE", 0}] = bit(enable);
        resource_type.pin_map.input_nodes[4095] = bit(lut_input) | bit(enable);
        resource().tile_type = &resource_type;
        cb->rememberNodeName(fpga::CB_NODE_LOCAL, root, "supply_one_64");
        cb->rememberNodeName(fpga::CB_NODE_LOCAL, enable, "enable_40");
        cb->rememberNodeName(fpga::CB_NODE_JOINT, gate, "junction_63");
        cb->rememberNodeName(fpga::CB_NODE_JOINT, relay, "junction_48");
        cb->constant_one_nodes.setBit(root);
        cb->local_joint[root].joint.setBit(gate);
        cb->joint_local[gate].local.setBit(enable);
        cb->joint_joint[gate].joint.setBit(relay);
        cb->joint_local[relay].local.setBit(lut_input);
        for (size_t i = 0; i < arrivals.size(); ++i) {
            incoming(arrivals[i], 1000 + i);
            cb->dst_joint[arrivals[i]].joint.setBit(gate);
        }
        incoming(alternate, 1100);
        cb->dst_local[alternate].local.setBit(lut_input);
        cb->rebuildOutgoingSrcs();

        auto* driver = cell("old_driver", "SOURCE", {{"O", rtl::Port::PORT_OUT}});
        old_lut = cell("old_lut", "LUT6", {{"I0", rtl::Port::PORT_IN}, {"I1", rtl::Port::PORT_IN},
            {"I2", rtl::Port::PORT_IN}, {"I3", rtl::Port::PORT_IN}, {"I4", rtl::Port::PORT_IN},
            {"I5", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
        old_net = &connect(*driver, *old_lut, "I4");
        require(resource().tryAddAt(old_lut, 3, false) == 3, "earlier LUT did not pack");
        route(*old_net, *driver, *old_lut, "I4", blocked ? 244 : alternate,
              blocked ? 1004 : 1100, lut_input, blocked);
        for (size_t i = 0; i < 14; ++i) {
            if (arrivals[i] == 244) continue;
            auto* d = cell("background_source_" + std::to_string(i), "SOURCE", {{"O", rtl::Port::PORT_OUT}});
            auto* s = cell("background_sink_" + std::to_string(i), "SINK", {{"I", rtl::Port::PORT_IN}});
            auto& n = connect(*d, *s, "I");
            route(n, *d, *s, "I", arrivals[i], 1000 + i, 200 + i, false);
        }
        cb->rebuildOutgoingSrcs();
        // Match placement-time RTL: a logical global constant, not an ordinary
        // positive-designator net fabricated to exercise the signal check.
        auto* one = cell("constant_one", "SOURCE", {{"O", rtl::Port::PORT_OUT}});
        auto* global = port(*one, "O");
        global->port_ref->is_global = true; global->port_ref->designator = -2;
        new_reg = cell("new_register", "FDRE", {{"CE", rtl::Port::PORT_IN}, {"Q", rtl::Port::PORT_OUT}});
        auto* ce = port(*new_reg, "CE");
        ce->port_ref->designator = -2; ce->set(global);
    }
    ~ConstantFixture() { device.tile_grid.clear(); device.cb_types.clear(); }

    void audit()
    {
        std::vector<rtl::Net*> nets;
        for (auto& net : top.nets) nets.push_back(&net);
        for (auto& tile : device.tile_grid) {
            std::ostringstream text;
            auto a = fpga::auditTileCongestion(tile, text, nets);
            if (a.orphan_leases || a.missing_leases || a.missing_registrations || a.stale_registrations)
                throw std::runtime_error(text.str());
        }
    }

    void verify()
    {
        require(device.routeTile(resource()) == &matrix(), "wrong resource-to-CB mapping");
        require(resource().getPinNodes("FDRE", "CE", 0) == bit(enable), "wrong CE mapping");
        require(cb->constant_one_nodes == bit(root) && cb->local_joint[root].joint == bit(gate)
                && cb->local_local[root].local == NodeMask{}, "wrong constant source topology");
        // Prove the unique final predecessor, including every LOCAL/JOINT,
        // rather than assuming the intended two-edge path has no bypass.
        for (int n = 0; n < CB_MAX_NODES; ++n) {
            require(!cb->local_local[n].local.testBit(enable), "unexpected direct enable path");
            require(cb->joint_local[n].local.testBit(enable) == (n == gate),
                    "unexpected enable JOINT predecessor");
        }
        require(!matrix().isPinNodeLeased(enable) && !matrix().cb.local.local.testBit(enable),
                "CE itself must be free");
        require(fpga::packedInputJointReservations(matrix()) == NodeMask{},
                "old LUT must not require a mandatory JOINT reservation");
        require(cb->canInAvoidingJoint(lut_input, gate), "old LUT has no bypass topology");
        require(!cb->canInAvoidingJoint(enable, gate), "CE has an unexpected bypass");
        int entries = 0, free_dsts = 0, free_paths = 0;
        for (const auto& e : cb->terminalEntries(enable)) {
            ++entries;
            require(e.joint == gate && e.joint2 < 0, "wrong CE terminal shape");
            auto found = std::find(arrivals.begin(), arrivals.end(), e.dst);
            require(found != arrivals.end(), "unexpected CE arrival");
            auto i = found - arrivals.begin();
            auto landing = device.resolveJump(source(), 1000 + i);
            require(landing.tile == &matrix() && landing.dst_node == e.dst, "invalid incoming mapping");
            free_dsts += !matrix().cb.dst.jump.testBit(e.dst);
            auto trial = matrix().cb;
            free_paths += trial.leaseIn(e.dst, enable, e.joint);
        }
        require(entries == 16 && free_dsts == (blocked ? 2 : 3), "wrong arrival occupancy split");
        require(free_paths == (blocked ? 0 : 3), "wrong CE path availability");
        for (auto& net : top.nets) for (const auto& b : net.routes) {
            const auto& wires = b.owner->wires[b.route_index];
            const auto& hop = wires[1];
            auto landing = device.resolveJump(source(), hop.jump);
            require(landing.tile == &matrix() && landing.dst_node == hop.dst, "invalid old jump");
            int joint = -1, first = -1;
            require(cb->canOut(hop.local, hop.jump, hop.jump, joint, &first), "invalid old Takeoff");
            require(cb->canIn(wires[2].local, wires[3].local, joint, &first), "invalid old Grounding");
            require(joint == wires[2].joint && first == wires[2].joint2, "wrong old JOINT sequence");
        }
        if (blocked) {
            require(fpga::findNetByNode(matrix(), fpga::CB_NODE_JOINT, gate, false) == old_net,
                    "missing live JOINT owner");
            require(!fpga::findNetByNode(matrix(), fpga::CB_NODE_JOINT, gate, true),
                    "old Grounding was classified as transit");
        }
        audit();
        std::printf("INCREMENTAL_CONSTANT blocked=%d entries=%d free_dsts=%d free_paths=%d "
                    "constant_path=%s audit=clean\n", blocked, entries, free_dsts, free_paths,
                    blocked ? "blocked_at_junction_63" : "free");
    }
};
}

void incremental_constant_packing_regression(bool snapshot_only)
{
    for (bool strict : {false, true}) {
        ConstantFixture f(false); f.verify();
        require(f.resource().peekAdd(f.new_reg, strict) == 0, "free constant path failed preview");
        require(!f.new_reg->tile.peer, "preview changed placement");
        require(f.resource().tryAddAt(f.new_reg, 0, strict) == 0, "free constant path failed packing");
        f.audit();
    }
    int failures = 0;
    for (int mode = 0; mode < (snapshot_only ? 1 : 4); ++mode) {
        ConstantFixture f; f.verify();
        if (snapshot_only) return;
        const bool strict = mode % 2;
        const int result = mode < 2 ? f.resource().peekAdd(f.new_reg, strict)
                                    : f.resource().tryAddAt(f.new_reg, 0, strict);
        std::printf("INCREMENTAL_CONSTANT operation=%s strict=%d result=%d expected=reject\n",
                    mode < 2 ? "preview" : "exact", strict, result);
        failures += result >= 0;
        if (mode < 2 || result < 0) require(!f.new_reg->tile.peer, "preview/rejection changed placement");
        require(f.old_lut->pos == 3 && f.old_lut->outline.fixed && f.old_lut->pnr_completed
                && fpga::isRouteComplete(f.old_lut->wires[0])
                && f.matrix().cb.joint.jump.testBit(ConstantFixture::gate),
                "packing damaged the earlier fixed Grounding");
        f.audit();
    }
    require(failures == 0,
            "incremental packing accepted constant-fed CE behind a foreign Grounding JOINT");

    // A live distributed route has a generated physical source, whereas the
    // unpacked CE still follows a logical global constant. Identity differs;
    // sharing is legal only when the two constant values match.
    for (bool same_value : {false, true}) {
        ConstantFixture f(false); f.verify();
        auto* physical = f.cell("physical_supply", "SOURCE", {{"O", rtl::Port::PORT_OUT}});
        auto* sink = f.cell("old_constant_sink", "SINK", {{"I", rtl::Port::PORT_IN}});
        auto& net = f.connect(*physical, *sink, "I");
        net.distributed_source = true; net.distributed_one = same_value;
        net.route_protected = true; net.pnr_preserved = true;
        constexpr int other_root = 66, other_input = 41;
        int root = same_value ? ConstantFixture::root : other_root;
        if (!same_value) f.cb->constant_zero_nodes.setBit(root);
        f.cb->local_joint[root].joint.setBit(ConstantFixture::gate);
        f.cb->joint_local[ConstantFixture::gate].local.setBit(other_input);
        f.cb->rememberNodeName(fpga::CB_NODE_LOCAL, other_root, "supply_zero_66");
        f.cb->rememberNodeName(fpga::CB_NODE_LOCAL, other_input, "enable_41");
        sink->tile.set(&f.resource()); sink->coord = f.resource().coord;
        fpga::Wire first;
        first.type = fpga::Wire::WIRE_ROUTE_EDGE;
        first.from = first.to = f.matrix().coord;
        first.from_node_type = fpga::CB_NODE_LOCAL; first.from_node = root;
        first.to_node_type = fpga::CB_NODE_JOINT; first.to_node = ConstantFixture::gate;
        fpga::Wire last = first;
        last.from_node_type = fpga::CB_NODE_JOINT; last.from_node = ConstantFixture::gate;
        last.to_node_type = fpga::CB_NODE_LOCAL; last.to_node = other_input;
        fpga::Wire pin;
        pin.type = fpga::Wire::WIRE_TILE_PIN; pin.from = pin.to = f.matrix().coord;
        pin.local = other_input; pin.pin_dir = 0;
        sink->wires.push_back({first, last, pin});
        f.matrix().cb.joint.jump.setBit(ConstantFixture::gate);
        f.matrix().cb.local.local.setBit(other_input);
        f.matrix().pin_state.leased_nodes.setBit(other_input);
        auto b = fpga::attachNetRoute(net, *sink, 0, physical, sink, "O", "I", net.name);
        fpga::registerNetRouteTiles(net, sink->wires[0], b);
        require(fpga::isRouteComplete(sink->wires[0]), "constant control route incomplete");
        f.audit();
        for (bool strict : {false, true})
            require((f.resource().peekAdd(f.new_reg, strict) == 0) == same_value,
                    "constant path sharing ignored polarity or physical/logical identity");
        require((f.resource().tryAddAt(f.new_reg, 0, false) == 0) == same_value,
                "exact constant path sharing ignored polarity");
        f.audit();
    }
    {
        ConstantFixture f;
        // A longer available path must remain usable despite the busy shortest
        // path. Include a cycle to check bounded graph traversal.
        f.cb->local_joint[ConstantFixture::root].joint.setBit(70);
        f.cb->joint_joint[70].joint.setBit(71);
        f.cb->joint_joint[71].joint.setBit(70);
        f.cb->joint_local[71].local.setBit(ConstantFixture::enable);
        f.cb->rememberNodeName(fpga::CB_NODE_JOINT, 70, "detour_70");
        f.cb->rememberNodeName(fpga::CB_NODE_JOINT, 71, "detour_71");
        require(f.resource().peekAdd(f.new_reg, false) == 0, "available constant bypass was ignored");
        require(f.resource().tryAddAt(f.new_reg, 0, false) == 0, "constant bypass failed exact packing");
        f.audit();
    }
    {
        ConstantFixture f(false);
        f.cb->constant_one_nodes = {};
        f.cb->constant_zero_nodes.setBit(ConstantFixture::root);
        require(f.resource().peekAdd(f.new_reg, false) < 0,
                "constant one was supplied from a constant-zero-only fabric");
        f.audit();
    }
    {
        ConstantFixture f(false);
        // Deliberately corrupt one lease: unknown ownership must not be
        // treated as same-constant sharing. Restore it before the audit.
        f.matrix().cb.joint.jump.setBit(ConstantFixture::gate);
        require(f.resource().peekAdd(f.new_reg, false) < 0, "unowned constant-path lease was ignored");
        f.matrix().cb.joint.jump &= ~bit(ConstantFixture::gate);
        f.audit();
    }
}
