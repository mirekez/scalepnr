#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Inst.h"
#include "Wire.h"
#include "route/RouteDesign.h"

#include <array>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace {
void check(bool ok, const std::string& reason) {
    if (!ok) throw std::runtime_error("moving blacklist mesh: " + reason);
}
NodeMask bit(int n) { NodeMask result{}; result.setBit(n); return result; }
using Key = std::tuple<int, int, int>;
using Route = std::vector<fpga::Wire>;

struct Owner {
    Referable<rtl::Cell> driver_cell, sink_cell;
    rtl::Inst driver, sink;
    Referable<rtl::Net> net;
};

// One default crossbar is shared by every tile. East/south rails form a
// converging 2D mesh: each source has both short and long physical landings.
// Alternating rails require one/two joints; occupancy comes from real routes.
struct Mesh {
    static constexpr int side = 8, input = 60, output = 61;
    fpga::Device& device = fpga::Device::current();
    std::vector<std::unique_ptr<Owner>> owners;
    std::vector<fpga::CBState> saved;
    std::vector<NodeMask> saved_pins;
    int dst_base, src_base, joint_base;

    explicit Mesh(unsigned seed) {
        device.tile_grid.clear();
        device.cb_types.clear();
        device.grid_spec.size = {side, side};
        device.size_width = device.size_height = side;
        device.tile_grid.resize(side * side);
        device.cb_types.resize(1);
        auto& cb = device.cb_types.front();
        cb.name = "matrix_701";
        cb.type_id = cb.base_type_id = 0;
        std::mt19937 random(seed);
        dst_base = 100 + random() % 400;
        src_base = 600 + random() % 400;
        joint_base = 1100 + random() % 400;
        cb.rememberNodeName(fpga::CB_NODE_LOCAL, input, "port_in_71");
        cb.rememberNodeName(fpga::CB_NODE_LOCAL, output, "port_out_72");
        for (int lane = 0; lane < 4; ++lane) {
            const int src = src_base + lane, dst = dst_base + lane;
            cb.rememberNodeName(fpga::CB_NODE_SRC, src, "rail_out_" + std::to_string(lane));
            cb.rememberNodeName(fpga::CB_NODE_DST, dst, "rail_in_" + std::to_string(lane));
            for (int length : {1, 2}) {
                fpga::Coord delta = lane < 2 ? fpga::Coord{length, 0} : fpga::Coord{0, length};
                cb.dst_by_src[src].push_back({delta, 0, {bit(dst)}, {}, true});
                cb.dst_by_src[src + 4].push_back({delta, 0, {bit(dst)}, {}, true});
            }
            // Injection-only rails have no incoming transit path. A rejected
            // placement can exhaust these before the second landing is visited.
            cb.rememberNodeName(fpga::CB_NODE_SRC, src + 4, "injection_" + std::to_string(lane));
            cb.local_src[output].jump.setBit(src + 4);
            cb.dst_local[dst].local.setBit(input);
            if (lane % 2 == 0) cb.local_src[output].jump.setBit(src);
            else {
                const int first = joint_base + lane * 2, last = first + 1;
                cb.rememberNodeName(fpga::CB_NODE_JOINT, first, "switch_a_" + std::to_string(lane));
                cb.rememberNodeName(fpga::CB_NODE_JOINT, last, "switch_b_" + std::to_string(lane));
                cb.local_joint[output].joint.setBit(first);
                if (lane == 1) cb.joint_src[first].jump.setBit(src);
                else {
                    cb.joint_joint[first].joint.setBit(last);
                    cb.joint_src[last].jump.setBit(src);
                }
            }
            for (int arrival = 0; arrival < 4; ++arrival) {
                if (lane % 2 == 0) cb.dst_src[dst_base + arrival].jump.setBit(src);
                else cb.dst_joint[dst_base + arrival].joint.setBit(joint_base + lane * 2);
            }
        }
        cb.rebuildOutgoingSrcs();
        for (int y = 0; y < side; ++y) for (int x = 0; x < side; ++x) {
            auto& t = tile(x, y);
            t.coord = t.cb_coord = {x, y};
            t.full_name = "square_" + std::to_string(x) + "_" + std::to_string(y);
            t.cb_type = t.cb.type = &cb;
            for (int lane = 0; lane < 4; ++lane)
                if (lane < 2 ? x > 0 : y > 0) t.incoming_dst_nodes.setBit(dst_base + lane);
        }
        // Keep the perimeter available for the later successful search, while
        // scattering independently owned direct/joint routes through the core.
        for (int y = 1; y < side - 2; ++y) for (int x = 1; x < side - 2; ++x) {
            if (random() % 3 != 0) continue;
            install(x, y, random() % 4);
        }
        check(!owners.empty(), "seed produced no occupied routes");
        snapshot();
        audit();
    }

    fpga::Tile& tile(int x, int y) { return *device.getTile(x, y); }
    fpga::Wire pin(fpga::Coord coord, int local, bool out) {
        fpga::Wire w;
        w.type = fpga::Wire::WIRE_TILE_PIN;
        w.from = w.to = w.resource = coord;
        w.local = local; w.pin_dir = out;
        return w;
    }
    Route terminal(fpga::Tile& t, int dst) {
        fpga::Wire w;
        w.from = w.to = t.coord; w.pos = 1; w.local = dst;
        return {w, pin(t.coord, input, false)};
    }
    void install(int x, int y, int lane) {
        auto& from = tile(x, y);
        auto& to = tile(x + (lane < 2), y + (lane >= 2));
        if (from.cb.local.local.testBit(output) || to.cb.local.local.testBit(input)) return;
        Route route{pin(from.coord, output, true)};
        fpga::Wire w;
        w.from = from.coord; w.to = to.coord; w.pos = 0; w.local = output;
        w.jump = w.route_jump = src_base + lane; w.dst = dst_base + lane;
        if (lane % 2) {
            w.joint = joint_base + lane * 2;
            if (lane == 3) { w.joint2 = w.joint; ++w.joint; }
        }
        route.push_back(w);
        auto tail = terminal(to, w.dst);
        route.insert(route.end(), tail.begin(), tail.end());
        installRoute(std::move(route));
    }
    void installRoute(Route route) {
        auto& from = tile(route.front().from.x, route.front().from.y);
        auto& to = tile(route.back().from.x, route.back().from.y);
        check(pnr::commitPreparedRoute(route), "cannot commit reference occupancy route");
        auto owner = std::make_unique<Owner>();
        owner->net.name = "occupied_signal_" + std::to_string(owners.size());
        owner->driver_cell.name = owner->net.name + "_driver";
        owner->sink_cell.name = owner->net.name + "_sink";
        owner->driver_cell.type = owner->sink_cell.type = "logic_702";
        owner->driver.cell_ref.set(&owner->driver_cell);
        owner->sink.cell_ref.set(&owner->sink_cell);
        owner->driver.tile.set(&device.tile_grid[from.coord.y * side + from.coord.x]);
        owner->sink.tile.set(&device.tile_grid[to.coord.y * side + to.coord.x]);
        owner->sink.wires.push_back(std::move(route));
        auto binding = fpga::attachNetRoute(owner->net, owner->sink, 0,
            &owner->driver, &owner->sink, "out", "in", owner->net.name);
        fpga::registerNetRouteTiles(owner->net, owner->sink.wires.front(), binding);
        owners.push_back(std::move(owner));
    }
    void roundTrip(Route route) {
        if (route.front().type != fpga::Wire::WIRE_TILE_PIN)
            route.insert(route.begin(), pin(route.front().from, output, true));
        installRoute(std::move(route));
        // A returned path must be usable, not merely a successful probe. Audit
        // committed leases and then remove exactly this new route's resources.
        audit();
        check(fpga::unrouteNet(owners.back()->net), "cannot unroute successful search result");
        owners.pop_back();
        unchanged();
    }
    void snapshot() {
        saved.clear(); saved_pins.clear();
        for (const auto& t : device.tile_grid) {
            saved.push_back(t.cb); saved_pins.push_back(t.pin_state.leased_nodes);
        }
    }
    void unchanged() {
        for (size_t i = 0; i < saved.size(); ++i) {
            const auto& t = device.tile_grid[i]; const auto& b = saved[i];
            // Every speculative SRC/DST/joint/local lease and deadend must be
            // restored, including occupied bits belonging to existing routes.
            check(t.cb.src.jump == b.src.jump && t.cb.dst.jump == b.dst.jump &&
                  t.cb.joint.jump == b.joint.jump && t.cb.local.local == b.local.local &&
                  t.cb.src_deadend.jump == b.src_deadend.jump &&
                  t.pin_state.leased_nodes == saved_pins[i], "search changed live occupancy");
        }
        audit();
    }
    void audit() {
        std::vector<rtl::Net*> nets;
        for (const auto& owner : owners) nets.push_back(&owner->net);
        const fpga::CongestionAuditIndex index(nets);
        size_t leased = 0;
        for (auto& t : device.tile_grid) {
            std::ostringstream text;
            auto a = fpga::auditTileCongestion(t, text, index);
            check(!a.orphan_leases && !a.missing_leases && !a.missing_registrations &&
                  !a.stale_registrations && !a.conflicting_driver_nodes,
                  "occupancy lacks a consistent owner: " + text.str());
            leased += a.leased_nodes;
        }
        check(leased > 0, "ownership audit saw no congestion");
    }
};

void uniqueProbes(const std::map<Key, size_t>& probes) {
    for (const auto& [key, count] : probes)
        // Different arrival/source nodes on the same tile remain distinct.
        check(count == 1, "exhausted node was placement-probed repeatedly");
}
} // namespace

void moving_destination_blacklist_grid() {
    for (unsigned seed = 1; seed <= 10; ++seed) {
        Mesh mesh(seed);
        std::map<Key, size_t> probes;
        bool one_joint_seen = false, two_joints_seen = false;
        const std::vector<pnr::ForwardPlacementAnchor> anchors{{&mesh.tile(0, 0), Mesh::output, false, {}}};
        auto reject = pnr::routeForwardToPlacement(anchors,
            [&](fpga::Tile& tile, int dst, bool, Route&) {
                ++probes[{tile.coord.x, tile.coord.y, dst}]; return false;
            }, {}, nullptr, [&](const auto&) {
                for (size_t i = 0; i < mesh.saved.size(); ++i) {
                    const auto added = mesh.device.tile_grid[i].cb.joint.jump & ~mesh.saved[i].joint.jump;
                    one_joint_seen |= added.testBit(mesh.joint_base + 2);
                    two_joints_seen |= added.testBit(mesh.joint_base + 6) && added.testBit(mesh.joint_base + 7);
                }
            });
        // Exhausting a converging mesh must cache failed DSTs instead of
        // re-enumerating the same subtrees through every different prefix.
        check(!reject.success && !reject.cancelled && reject.failed_states > 50 &&
              reject.failure_cache_hits > 50, "forward mesh did not exercise failure reuse");
        uniqueProbes(probes);
        // Convergence cannot turn a per-node blacklist into a per-tile ban;
        // all four different arrival rails at the far corner must be examined.
        for (int lane = 0; lane < 4; ++lane)
            check(probes.count({7, 7, mesh.dst_base + lane}), "distinct DST was blacklisted with its tile");
        check(one_joint_seen && two_joints_seen, "search did not exercise both joint path kinds");
        check(probes.size() <= 4 * Mesh::side * Mesh::side + 1,
              "forward work exceeded the number of physical arrival states");
        mesh.unchanged();
        std::cout << "destination seed=" << seed << " owners=" << mesh.owners.size()
                  << " probes=" << probes.size() << " expanded=" << reject.expanded
                  << " failed=" << reject.failed_states << " avoided_visits="
                  << reject.failure_cache_hits << '\n';
        auto success = pnr::routeForwardToPlacement(anchors,
            [&](fpga::Tile& tile, int dst, bool, Route& tail) {
                if (tile.coord.x != 7 || tile.coord.y != 7) return false;
                tail = mesh.terminal(tile, dst); return true;
            });
        // A new moving attempt must not inherit the previous call's blacklist.
        check(success.success && !success.fragments.empty(), "forward fresh call lost a reachable placement");
        mesh.unchanged();
        mesh.roundTrip(success.fragments);
        size_t count = 0;
        auto cancelled = pnr::routeForwardToPlacement(anchors,
            [&](auto&, int, bool, auto&) { ++count; return false; }, [&] { return count >= 3; });
        // Aborting a partially explored branch is not proof that it is dead.
        check(cancelled.cancelled && !cancelled.success && !cancelled.failed_states,
              "forward cancellation cached pending work");
        mesh.unchanged();
    }
}

void moving_source_blacklist_grid() {
    for (unsigned seed = 1; seed <= 10; ++seed) {
        Mesh mesh(seed);
        std::map<Key, size_t> probes;
        bool accept = false;
        size_t cancel_after = 0;
        auto run = [&] {
            return pnr::routeBackwardToTakeoff(mesh.tile(7, 7), bit(Mesh::input), {0, 0},
                0, 20, 20,
                [&](fpga::Tile& tile, int src, pnr::BackwardTakeoffChoice& choice) {
                    ++probes[{tile.coord.x, tile.coord.y, src}];
                    if (!accept || tile.coord.x != 0 || tile.coord.y != 0) return false;
                    const int lane = src - mesh.src_base;
                    choice.local = Mesh::output;
                    if (lane < 4 && lane % 2) {
                        choice.joint = mesh.joint_base + lane * 2;
                        if (lane == 3) { choice.joint2 = choice.joint; ++choice.joint; }
                    }
                    return true;
                }, nullptr, [&] { return cancel_after && probes.size() >= cancel_after; },
                0, 0, 0, nullptr, nullptr, {}, {}, [](const auto&) { return false; });
        };
        auto reject = run();
        // The same physical SRC is offered by its short and long landings.
        // Cache only after both the placement probe and predecessor search fail.
        check(!reject.success && reject.failed_states > 50 && reject.failure_cache_hits > 0,
              "backward mesh did not exercise exhausted-source reuse: failed=" +
              std::to_string(reject.failed_states) + " hits=" + std::to_string(reject.failure_cache_hits));
        uniqueProbes(probes);
        for (int lane = 0; lane < 8; ++lane)
            check(probes.count({0, 0, mesh.src_base + lane}), "distinct SRC was blacklisted with its tile");
        check(probes.size() <= 8 * Mesh::side * Mesh::side,
              "backward work exceeded the number of physical source states");
        mesh.unchanged();
        std::cout << "source seed=" << seed << " owners=" << mesh.owners.size()
                  << " probes=" << probes.size() << " expanded=" << reject.expanded
                  << " failed=" << reject.failed_states << " avoided_visits="
                  << reject.failure_cache_hits << '\n';
        probes.clear(); accept = true;
        auto success = run();
        // Live predecessor branches cannot be blacklisted just because a nearer
        // source placement failed; the far corner must remain discoverable.
        check(success.success && success.source_tile == &mesh.tile(0, 0) &&
              !success.fragments.empty(), "backward fresh call lost a reachable source");
        mesh.unchanged();
        mesh.roundTrip(success.fragments);
        probes.clear(); accept = false; cancel_after = 1;
        auto cancelled = run();
        check(!cancelled.success && !cancelled.failed_states,
              "backward cancellation cached pending work");
        mesh.unchanged();
    }
}
