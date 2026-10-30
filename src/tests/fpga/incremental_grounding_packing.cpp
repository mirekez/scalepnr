#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Module.h"
#include "Wire.h"
#include "route/MovingPlacementBlacklist.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace {
void check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

NodeMask bit(int node) { return NodeMask{0, 1} << node; }

// Reduced replay of the later-run packing failure. Coordinates are translated
// into three Tiles, but the terminal fan-in, occupied/free split, two-JOINT
// Grounding and ownership/reservation distinction match the failing snapshot.
// All routing node names below are abstract, not imported device wire names.
struct Fixture {
    static constexpr int gate = 23, relay = 9, reg_input = 1, lut_input = 115;
    static constexpr int alternate = 90;
    static constexpr std::array<int, 16> arrivals{
        19, 23, 64, 193, 242, 245, 257, 260,
        544, 737, 1024, 3072, 3616, 3809, 3840, 3846};
    static constexpr std::array<int, 9> other_busy{
        19, 193, 245, 260, 737, 3072, 3616, 3809, 3846};

    fpga::Device& device = fpga::Device::current();
    fpga::TileType resource_type{"resource_bank_731", 1, 0};
    fpga::CBType* cb = nullptr;
    Referable<rtl::Module> top, primitive;
    std::vector<std::unique_ptr<Referable<rtl::Cell>>> cells;
    std::vector<std::unique_ptr<Referable<rtl::Inst>>> insts;
    Referable<rtl::Inst>* old_lut = nullptr;
    Referable<rtl::Inst>* new_reg = nullptr;
    rtl::Net* old_net = nullptr;
    bool contested;

    Referable<fpga::Tile>& source() { return device.tile_grid[0]; }
    Referable<fpga::Tile>& matrix() { return device.tile_grid[1]; }
    Referable<fpga::Tile>& resource() { return device.tile_grid[2]; }

    Referable<rtl::Inst>* cell(const std::string& name, const std::string& type,
                              std::initializer_list<std::pair<const char*, int>> ports)
    {
        auto c = std::make_unique<Referable<rtl::Cell>>();
        c->name = name; c->type = type; c->module_ref.set(&primitive);
        for (auto [name, direction] : ports) {
            auto& port = c->ports.emplace_back();
            port.name = name; port.type = static_cast<decltype(port.type)>(direction);
        }
        auto i = std::make_unique<Referable<rtl::Inst>>();
        i->cell_ref.set(c.get()); i->pos = -1;
        i->cnt_inputs = i->cnt_outputs = 0;
        for (auto& port : c->ports) {
            auto& conn = i->conns.emplace_back();
            conn.port_ref.set(&port); conn.inst_ref.set(i.get());
            i->cnt_inputs += port.type == rtl::Port::PORT_IN;
            i->cnt_outputs += port.type == rtl::Port::PORT_OUT;
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

    // Install a complete, topology-valid earlier-run route, not anonymous busy
    // mask bits. All leases are accompanied by registered owning fragments.
    void route(rtl::Net& net, rtl::Inst& driver, rtl::Inst& sink,
               const std::string& input, int dst, int src, int local, bool two_joints)
    {
        int output = src - 700;
        cb->local_src[output].jump.setBit(src);
        cb->rememberNodeName(fpga::CB_NODE_LOCAL, output, "output_" + std::to_string(output));
        cb->rememberNodeName(fpga::CB_NODE_LOCAL, local, "input_" + std::to_string(local));
        if (!two_joints) cb->dst_local[dst].local.setBit(local);
        driver.tile.set(&source()); driver.coord = source().coord;
        // The surrounding routes have endpoints in the same resource Tile but
        // need no packing slots in this reduced fixture; only the LUT/register
        // pair under test participates in Element packing.
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
        if (two_joints) { ground.joint = relay; ground.joint2 = gate; }
        fpga::Wire pin;
        pin.type = fpga::Wire::WIRE_TILE_PIN;
        pin.from = pin.to = matrix().coord; pin.local = local; pin.pin_dir = 0;
        sink.wires.push_back({out, hop, ground, pin});
        source().cb.src.jump.setBit(src);
        matrix().cb.dst.jump.setBit(dst);
        matrix().cb.local.local.setBit(local);
        matrix().pin_state.leased_nodes.setBit(local);
        if (two_joints) {
            matrix().cb.joint.jump.setBit(gate); matrix().cb.joint.jump.setBit(relay);
        }
        net.pnr_preserved = true;
        auto binding = fpga::attachNetRoute(net, sink, 0, &driver, &sink, "O", input, net.name);
        fpga::registerNetRouteTiles(net, sink.wires[0], binding);
        check(fpga::isRouteComplete(sink.wires[0]), "earlier-run fixture route is incomplete");
    }

    explicit Fixture(bool blocked = true) : contested(blocked)
    {
        device.tile_grid.clear(); device.cb_types.clear();
        device.cb_types.emplace_back(); cb = &device.cb_types.back();
        cb->name = "matrix_731"; cb->type_id = cb->base_type_id = 0;
        // The fourth Tile is only used by the transit/Takeoff controls.
        device.size_width = 4; device.size_height = 1; device.grid_spec.size = {4, 1};
        device.tile_grid.resize(4);
        for (int x = 0; x < 4; ++x) {
            auto& tile = device.tile_grid[x];
            tile.coord = tile.name = {x, 0}; tile.cb_coord = x == 2 ? fpga::Coord{1, 0} : tile.coord;
            tile.cb_type = cb; tile.cb.type = cb;
        }
        top.name = "test_top"; top.nets.reserve(32);
        primitive.name = "primitive"; primitive.is_blackbox = true; primitive.parent_ref.set(&top);
        for (auto type : {fpga::ELEMENT_LUT5, fpga::ELEMENT_LUT1, fpga::ELEMENT_FD}) {
            fpga::Element e;
            e.name = "slot_" + std::to_string(type); e.type = type;
            e.bitmap_pos = 0; e.elements_to_left = static_cast<int>(type);
            resource_type.elements.push_back(e);
        }
        // Explicit abstract cell/port/position maps avoid device-specific pin names.
        resource_type.pin_map.nodes[{"LUT6", "I5", 3}] = bit(lut_input);
        resource_type.pin_map.nodes[{"FDRE", "D", 0}] = bit(reg_input);
        resource_type.pin_map.input_nodes[4095] = bit(lut_input) | bit(reg_input);
        resource().tile_type = &resource_type;
        cb->rememberNodeName(fpga::CB_NODE_JOINT, gate, "junction_23");
        cb->rememberNodeName(fpga::CB_NODE_JOINT, relay, "junction_9");
        cb->rememberNodeName(fpga::CB_NODE_LOCAL, reg_input, "input_1");
        for (size_t i = 0; i < arrivals.size(); ++i) {
            incoming(arrivals[i], 1000 + i);
            cb->dst_joint[arrivals[i]].joint.setBit(gate);
        }
        incoming(alternate, 1100);
        cb->joint_local[gate].local.setBit(reg_input);
        cb->joint_joint[gate].joint.setBit(relay);
        cb->joint_local[relay].local.setBit(lut_input);
        cb->dst_local[alternate].local.setBit(lut_input);
        cb->rebuildOutgoingSrcs();

        auto* old_driver = cell("old_driver", "SOURCE", {{"O", rtl::Port::PORT_OUT}});
        old_lut = cell("old_lut", "LUT6", {{"I0", rtl::Port::PORT_IN}, {"I1", rtl::Port::PORT_IN},
            {"I2", rtl::Port::PORT_IN}, {"I3", rtl::Port::PORT_IN}, {"I4", rtl::Port::PORT_IN},
            {"I5", rtl::Port::PORT_IN}, {"O", rtl::Port::PORT_OUT}});
        old_net = &connect(*old_driver, *old_lut, "I5");
        check(resource().tryAddAt(old_lut, 3, false) == 3, "earlier-run LUT did not pack");
        route(*old_net, *old_driver, *old_lut, "I5", blocked ? 23 : alternate,
              blocked ? 1001 : 1100, lut_input, blocked);
        for (size_t i = 0; i < other_busy.size(); ++i) {
            auto* driver = cell("background_source_" + std::to_string(i), "SOURCE", {{"O", rtl::Port::PORT_OUT}});
            auto* sink = cell("background_sink_" + std::to_string(i), "SINK", {{"I", rtl::Port::PORT_IN}});
            auto& net = connect(*driver, *sink, "I");
            size_t arrival_index = std::find(arrivals.begin(), arrivals.end(), other_busy[i]) - arrivals.begin();
            route(net, *driver, *sink, "I", other_busy[i], 1000 + arrival_index, 200 + i, false);
        }
        cb->rebuildOutgoingSrcs();
        auto* new_driver = cell("new_driver", "SOURCE", {{"O", rtl::Port::PORT_OUT}});
        new_reg = cell("new_register", "FDRE", {{"D", rtl::Port::PORT_IN}, {"Q", rtl::Port::PORT_OUT}});
        connect(*new_driver, *new_reg, "D");
    }

    ~Fixture() { device.tile_grid.clear(); device.cb_types.clear(); }

    void makeOldRouteContinue(bool takeoff)
    {
        auto wires = old_lut->wires[0];
        auto* driver = old_net->routes[0].from;
        check(fpga::unrouteNet(*old_net), "control could not release old Grounding");
        check(resource().unassign(old_lut), "control could not detach old LUT");
        auto& remote = device.tile_grid[3];
        remote.tile_type = &resource_type;
        check(remote.tryAddAt(old_lut, 3, false) == 3, "control could not pack remote LUT");
        constexpr int exit = 1200, landing = 91;
        cb->joint_src[relay].jump.setBit(exit);
        cb->dst_local[landing].local.setBit(lut_input);
        cb->rememberNodeName(fpga::CB_NODE_SRC, exit, "departure_1200");
        cb->rememberNodeName(fpga::CB_NODE_DST, landing, "arrival_91");
        fpga::CBType::ResolvedJump jump{};
        jump.delta = {2, 0}; jump.target_cb_type_id = 0; jump.dsts.jump.setBit(landing);
        cb->dst_by_src[exit].push_back(jump);
        remote.incoming_dst_nodes.setBit(landing);
        auto& through = wires[2];
        through.to = remote.coord; through.jump = exit; through.dst = landing;
        if (takeoff) {
            driver->tile.set(&matrix()); driver->coord = matrix().coord;
            wires[0].from = wires[0].to = matrix().coord;
            through.pos = 0; through.local = wires[0].local;
            cb->local_joint[through.local].joint.setBit(gate);
            wires.erase(wires.begin() + 1);
        } else {
            source().cb.src.jump.setBit(wires[1].jump);
            matrix().cb.dst.jump.setBit(23);
        }
        fpga::Wire terminal;
        terminal.from = terminal.to = remote.coord; terminal.pos = 1; terminal.local = landing;
        wires.back().from = wires.back().to = remote.coord;
        wires.insert(wires.end() - 1, terminal);
        old_lut->wires[0] = std::move(wires);
        matrix().cb.src.jump.setBit(exit);
        matrix().cb.joint.jump.setBit(gate); matrix().cb.joint.jump.setBit(relay);
        remote.cb.dst.jump.setBit(landing); remote.cb.local.local.setBit(lut_input);
        remote.pin_state.leased_nodes.setBit(lut_input);
        cb->rebuildOutgoingSrcs();
        auto binding = fpga::attachNetRoute(*old_net, *old_lut, 0, driver, old_lut,
                                           "O", "I5", old_net->name);
        fpga::registerNetRouteTiles(*old_net, old_lut->wires[0], binding);
        check(fpga::isRouteComplete(old_lut->wires[0]), "control route is incomplete");
        auto resolved = device.resolveJump(matrix(), exit);
        check(resolved.tile == &remote && resolved.dst_node == landing, "control jump does not resolve");
        check(bool(fpga::findNetByNode(matrix(), fpga::CB_NODE_JOINT, gate, true)) == !takeoff,
              "control JOINT owner is misclassified");
        audit();
    }

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
        check(device.routeTile(resource()) == &matrix(), "resource is not attached to the destination CB");
        check(resource().getPinNodes("FDRE", "D", 0) == bit(reg_input), "register endpoint mapping differs");
        check(resource().getPinNodes("LUT6", "I5", 3) == bit(lut_input), "LUT endpoint mapping differs");
        check(fpga::packedInputJointReservations(matrix()) == NodeMask{},
              "old LUT must have no mandatory reservation: its second path avoids the joint");
        check(cb->canInAvoidingJoint(lut_input, gate), "LUT lost its alternative physical input path");
        check(!cb->canInAvoidingJoint(reg_input, gate), "register acquired an unintended bypass");
        check(!matrix().isPinNodeLeased(reg_input), "new register input is already occupied");
        // Prove that the leases/owners describe actual traversable topology,
        // rather than merely internally consistent bookkeeping.
        for (auto& net : top.nets) for (const auto& binding : net.routes) {
            const auto& wires = binding.owner->wires[binding.route_index];
            const auto& hop = wires[1];
            auto landing = device.resolveJump(source(), hop.jump);
            check(landing.tile == &matrix() && landing.dst_node == hop.dst,
                  "earlier route has an invalid inter-Tile jump");
            int joint = -1, first_joint = -1;
            check(cb->canOut(hop.local, hop.jump, hop.jump, joint, &first_joint),
                  "earlier route has an invalid source Takeoff");
            check(cb->canIn(wires[2].local, wires[3].local, joint, &first_joint),
                  "earlier route has an invalid Grounding");
            check(joint == wires[2].joint && first_joint == wires[2].joint2,
                  "earlier route joint sequence does not match the topology");
        }
        int entries = 0, free_dsts = 0, free_paths = 0;
        for (const auto& e : cb->terminalEntries(reg_input)) {
            ++entries;
            check(e.joint == gate && e.joint2 < 0, "register terminal differs from the recorded topology");
            check(matrix().incoming_dst_nodes.testBit(e.dst), "terminal has no incoming mapping");
            auto index = std::find(arrivals.begin(), arrivals.end(), e.dst) - arrivals.begin();
            auto landing = device.resolveJump(source(), 1000 + index);
            check(landing.tile == &matrix() && landing.dst_node == e.dst,
                  "register terminal incoming mapping does not resolve");
            free_dsts += !matrix().cb.dst.jump.testBit(e.dst);
            auto trial = matrix().cb;
            free_paths += trial.leaseIn(e.dst, reg_input, e.joint);
        }
        check(entries == 16, "expected all sixteen register terminal entries");
        check(free_dsts == (contested ? 6 : 7), "incorrect occupied/free DST split");
        check(free_paths == (contested ? 0 : 7), "terminal lease result does not reproduce the obstruction");
        if (contested) {
            check(fpga::findNetByNode(matrix(), fpga::CB_NODE_JOINT, gate, false) == old_net,
                  "shared joint is not owned by the earlier route");
            check(!fpga::findNetByNode(matrix(), fpga::CB_NODE_JOINT, gate, true),
                  "Grounding was incorrectly classified as preemptible transit");
        }
        audit();
        std::printf("INCREMENTAL_GROUNDING snapshot blocked=%d entries=%d free_dsts=%d free_paths=%d audit=clean\n",
                    contested, entries, free_dsts, free_paths);
    }
};
}

void incremental_grounding_packing_regression(bool snapshot_only)
{
    // Control: the very same cells and surrounding occupancy work when the
    // older LUT uses its alternative path. Do not solve this by banning all
    // placement beside preserved routes or rejecting the resource type.
    {
        Fixture f(false); f.verify();
        check(f.resource().peekAdd(f.new_reg, false) == 0, "free-path control failed preview packing");
        check(f.resource().tryAddAt(f.new_reg, 0, false) == 0, "free-path control failed exact packing");
        f.audit();
    }
    int failures = 0;
    for (int mode = 0; mode < (snapshot_only ? 1 : 3); ++mode) {
        Fixture f; f.verify();
        if (snapshot_only) return;
        int result = mode == 0 ? f.resource().peekAdd(f.new_reg, false)
                   : mode == 1 ? f.resource().peekAdd(f.new_reg, true)
                               : f.resource().tryAddAt(f.new_reg, 0, false);
        const char* operation = mode == 0 ? "peek_relaxed" : mode == 1 ? "peek_strict" : "commit_relaxed";
        std::printf("INCREMENTAL_GROUNDING operation=%s result=%d expected=reject\n", operation, result);
        failures += result >= 0;
        if (mode < 2) check(!f.new_reg->tile.peer, "preview unexpectedly placed the register");
        check(f.old_lut->pnr_completed && f.old_lut->outline.fixed && f.old_lut->pos == 3,
              "packing changed the earlier fixed LUT");
        check(f.matrix().cb.joint.jump.testBit(Fixture::gate) &&
              fpga::isRouteComplete(f.old_lut->wires[0]), "packing damaged the earlier Grounding");
        f.audit();
    }
    check(!failures, "incremental packing accepted a register whose mandatory JOINT is occupied by preserved Grounding");
    {
        Fixture f;
        auto* driver = f.old_net->routes[0].from;
        auto* input = f.port(*f.new_reg, "D"); auto* output = f.port(*driver, "O");
        input->set(output); input->port_ref->designator = output->port_ref->designator;
        check(f.resource().peekAdd(f.new_reg, false) == 0,
              "same-signal branch cannot share its existing JOINT");
        f.audit();
    }
    for (bool takeoff : {false, true}) {
        Fixture f; f.makeOldRouteContinue(takeoff);
        int result = f.resource().peekAdd(f.new_reg, false);
        check(takeoff ? result < 0 : result == 0,
              "live JOINT check must reject Takeoff but allow preserved transit");
        if (!takeoff) {
            f.old_net->route_protected = true;
            check(f.resource().peekAdd(f.new_reg, false) < 0,
                  "protected transit was treated as preemptible");
        }
        f.audit();
    }
}

// A real leased mandatory terminal belongs to another net, but is not evidence
// that the resource tile cannot pack this cell independently of its route.
void moving_blacklist_ignores_live_terminal_occupation()
{
    Fixture f;
    f.verify();
    pnr::MovingPlacementBlacklist blacklist({{f.new_reg}});
    check(f.resource().peekAdd(f.new_reg, false) < 0,
          "fixture must fail the ordinary relaxed live-route packing check");
    for (int attempt = 0; attempt < 3; ++attempt) {
        check(!blacklist.rejects(f.resource()) && blacklist.size() == 0,
              "live terminal congestion poisoned the packing blacklist");
        check(!f.new_reg->tile.peer && f.new_reg->pos == -1,
              "blacklist left the hypothetical cell placed");
        check(f.resource().peekAdd(f.new_reg, false) < 0,
              "placement-only preview leaked its bypass into normal packing");
        f.audit(); // Every original occupied node must still have its original owner.
    }
}
