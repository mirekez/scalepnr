#include "Timings.h"
#include "Tech.h"
#include "debug.h"
#include "on_return.h"

#include <cmath>
#include <numeric>
#include <stdexcept>

using namespace clk;

// A clock net used as data is not a clock pin. Buffer forks are visited once.
void Timings::recurseClockPeers(std::vector<TimingInfo>* infos,
    Referable<rtl::Conn>& conn, int depth, Referable<rtl::Conn>* root)
{
    if (depth == 0) {
        visited_clock.clear();
        visited_data.clear();
    }
    if (!visited_clock.insert(&conn).second) return;
    if (!conn.getPeers().empty()) {
        for (auto* peer : conn.getPeers())
            recurseClockPeers(infos, rtl::Conn::fromBase(*peer), depth + 1, root);
        return;
    }
    auto* inst = conn.inst_ref.peer;
    if (!inst || !inst->cell_ref.peer || !conn.port_ref.peer) return;
    const auto& type = inst->cell_ref->type;
    auto [first, last] = tech->buffers_ports.equal_range(type);
    if (conn.port_ref->type == rtl::Port::PORT_IN)
        for (; first != last; ++first)
            for (auto& output : inst->conns)
                if (output.port_ref->name == first->second
                    && output.port_ref->type == rtl::Port::PORT_OUT)
                    recurseClockPeers(infos, output, depth + 1, root);
    if (!tech->check_clocked(inst->cell_ref->type, conn.port_ref->name)) return;
    for (auto& data : inst->conns) {
        if (data.port_ref->type == rtl::Port::PORT_IN
            && !tech->check_clocked(inst->cell_ref->type, data.port_ref->name)
            && visited_data.insert(&data).second)
            infos->push_back(TimingInfo{.data_in = &data});
    }
}

bool Timings::recurseDataPeers(Referable<TimingPath>* path, int depth)
{
    if (depth == 0 && !building_clocks) outputs.clear();
    rtl::Conn* curr = path->data_in ? path->data_in->follow() : nullptr;
    if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox
        || curr->port_ref->is_global) return false;
    auto* inst = curr->inst_ref.peer;
    if (inst->locked) {
        PNR_WARNING("combinational loop at '{}'", curr->makeName());
        return false;
    }
    if (auto it = outputs.find(curr); it != outputs.end()) {
        path->precalculated = it->second;
        path->min_length = it->second->min_length;
        path->max_length = it->second->max_length;
        return true;
    }
    path->data_output = curr;
    bool clocked = tech->clocked_ports.contains(inst->cell_ref->type);
    if (clocked || tech->buffers_ports.contains(inst->cell_ref->type)) {
        if (clocked && building_clocks && capture_clock) {
            rtl::Clock* launch = nullptr;
            for (auto& conn : inst->conns)
                if (tech->check_clocked(inst->cell_ref->type, conn.port_ref->name)) {
                    auto* clock = building_clocks->findClock(&conn, tech->buffers_ports);
                    if (launch && clock && launch != clock) {
                        PNR_WARNING("unsupported multiple clock pins on '{}'", inst->makeName());
                        path->data_output = nullptr;
                        return false;
                    }
                    if (clock) launch = clock;
                }
            // An unconstrained launching register has no known launch edge.
            if (!launch || building_clocks->asynchronous(*launch, *capture_clock)) {
                path->data_output = nullptr;
                return false;
            }
            path->launch_clock = launch;
            // Primary clocks have rising edges at zero, with 1 fs resolution.
            auto launch_ticks = std::llround(launch->period_ns * 1e6);
            auto capture_ticks = std::llround(capture_clock->period_ns * 1e6);
            double separation = std::gcd(launch_ticks, capture_ticks) / 1e6;
            path->launch_offset_ns = capture_clock->period_ns - separation;
        }
        path->min_length = path->max_length = 0;
        return true;
    }
    inst->locked = true;
    on_return unlock([inst]() { inst->locked = false; });
    int minimum = 1000000000, maximum = -1;
    path->sub_paths.reserve(std::count_if(inst->conns.begin(), inst->conns.end(),
        [](auto& conn) { return conn.port_ref->type == rtl::Port::PORT_IN; }));
    for (auto& conn : inst->conns) {
        if (conn.port_ref->type != rtl::Port::PORT_IN) continue;
        auto& input = path->sub_paths.emplace_back(TimingPath{.data_in = &conn});
        if (!recurseDataPeers(&input, depth + 1)) {
            path->sub_paths.pop_back();
        } else {
            minimum = std::min(minimum, input.min_length);
            maximum = std::max(maximum, input.max_length);
        }
    }
    if (path->sub_paths.empty()) {
        path->data_output = nullptr;
        return false;
    }
    path->min_length = minimum + 1;
    path->max_length = maximum + 1;
    outputs.emplace(curr, path);
    return true;
}

void Timings::makeTimingsList(rtl::Design& design, Clocks& clocks)
{
    (void)design;
    decltype(clocked_inputs) next_inputs;
    // Complete endpoint vectors before saving any pointers into their paths.
    for (auto& clock : clocks.clocks_list)
        if (clock.conn_ptr) recurseClockPeers(&next_inputs[&clock], *clock.conn_ptr);
    std::unordered_map<rtl::Conn*, rtl::Clock*> captures;
    for (auto& [clock, infos] : next_inputs) for (auto& info : infos) {
        auto [it, inserted] = captures.emplace(info.data_in, clock);
        if (!inserted && it->second != clock)
            throw std::runtime_error("ambiguous capture clocks for " + info.data_in->makeName()
                + ": a multi-clock primitive needs per-port timing arcs");
    }
    building_clocks = &clocks;
    // A shared COMB cone may have different launch edges/exclusions per capture
    // domain. Share only inside that domain, keyed by the actual output port.
    for (auto& clock : clocks.clocks_list) {
        capture_clock = &clock;
        outputs.clear();
        for (auto& info : next_inputs[&clock]) {
            info.path.data_in = info.data_in;
            info.constrained = recurseDataPeers(&info.path);
        }
    }
    outputs.clear();
    visited_clock.clear();
    visited_data.clear();
    capture_clock = nullptr;
    building_clocks = nullptr;
    clocked_inputs.swap(next_inputs);
}

// calculate timings for one clock
void Timings::recurseTimings(Referable<TimingPath>& path, int depth)
{
    rtl::Conn* curr = path.data_in;
    PNR_LOG2_("CLKT", depth, "calculating net '{}' from '{}', depth '{}'", curr->makeNetName(), curr->makeName(), depth);

    if (path.precalculated) {  // this node was already calculated (if we didnt change traversal order!)
        path.max_setup_time = path.precalculated->max_setup_time;
        path.max_hold_time = path.precalculated->max_hold_time;
        path.min_setup_time = path.precalculated->min_setup_time;
        path.min_hold_time = path.precalculated->min_hold_time;
        PNR_LOG2_("CLKT", depth, "already calculated, setup: {:.3f}/{:.3f}, hold: {:.3f}/{:.3f}", path.max_setup_time, path.min_setup_time, path.max_hold_time, path.min_hold_time);
        return;
    }

    if (!path.data_output) {  // source from outside of top module
        path.max_setup_time = 0;  // todo: we need to add pins (IBUF) delay here
        path.max_hold_time = 0;
        path.min_setup_time = 0;
        path.min_hold_time = 0;
        return;
    }

    if (!path.sub_paths.size()) {
        path.max_setup_time = path.launch_offset_ns;  // no clock-to-Q model yet
        path.max_hold_time = 0;
        path.min_setup_time = path.launch_offset_ns;
        path.min_hold_time = 0;
        return;
    }

    double max_setup_time = 0;
    double max_hold_time = 0;
    double min_setup_time = 100000;
    double min_hold_time = 100000;
    for (auto& sub_path : path.sub_paths) {
        if (!sub_path.data_in) {
            continue;
        }

        int index_in = sub_path.data_in->port_ref->index;
        int index_out = path.data_output->port_ref->index;
        sub_path.own_setup_time = tech->comb_delays.getDelay(path.data_output->inst_ref->cell_ref->type, index_in, index_out);
        sub_path.own_hold_time = 0.0;  // not implemented yet

        recurseTimings(sub_path, depth + 1);

        if (sub_path.max_setup_time + sub_path.own_setup_time > max_setup_time) {
            max_setup_time = sub_path.max_setup_time + sub_path.own_setup_time;
        }
        if (sub_path.max_hold_time + sub_path.own_hold_time > max_hold_time) {
            max_hold_time = sub_path.max_hold_time + sub_path.own_hold_time;
        }
        if (sub_path.min_setup_time + sub_path.own_setup_time < min_setup_time) {
            min_setup_time = sub_path.min_setup_time + sub_path.own_setup_time;
        }
        if (sub_path.min_hold_time + sub_path.own_hold_time < min_hold_time) {
            min_hold_time = sub_path.min_hold_time + sub_path.own_hold_time;
        }
    }

    path.max_setup_time = max_setup_time;  // todo: add FD input delay here
    path.max_hold_time = max_hold_time;
    path.min_setup_time = min_setup_time;
    path.min_hold_time = min_hold_time;
    PNR_LOG2_("CLKT", depth, "result, setup: {:.3f}/{:.3f}, hold: {:.3f}/{:.3f}", path.max_setup_time, path.min_setup_time, path.max_hold_time, path.min_hold_time);
}

// calculate all timings
void Timings::calculateTimings()
{
    PNR_LOG1("CLKT", "calculateTimings");
    for (auto& clock : clocked_inputs) {
        PNR_LOG1("CLKT", "clock '{}' ('{}')", clock.first->name, clock.first->conn_name);
        for (auto& tinfo : clock.second) {
            if (!tinfo.constrained) continue;
            recurseTimings(tinfo.path);
        }
    }
}
