#include "Timings.h"
#include "debug.h"
#include "getInsts.h"
#include "on_return.h"

using namespace clk;

// finds all data inputs of clocked instances by this clock
void Timings::recurseClockPeers(std::vector<TimingInfo>* infos, Referable<rtl::Conn>& clk_conn, int depth, Referable<rtl::Conn>* root)
{
    if (root == 0) {
        root = &clk_conn;
        PNR_LOG1("CLKT", "recurseClockPeers, root conn: '{}'", root->makeName());
    }
    if (clk_conn.peers.size() == 0) {  // it's CLK input (should be BUFG or IBUF or error)
        auto it = tech->buffers_ports.find(clk_conn.inst_ref->cell_ref->type);
        if (it == tech->buffers_ports.end()) {
            PNR_LOG2("CLKT", "skipping '{}'", clk_conn.makeName());
            if (!clk_conn.inst_ref->cell_ref->module_ref->is_blackbox) {
                PNR_WARNING("floating clock net: got terminal conn '{}' ('{}') from clock input '{}', but it's not a is_blackbox ('{}')",
                    clk_conn.makeName(), clk_conn.inst_ref->cell_ref->type, root->makeName(), clk_conn.inst_ref->cell_ref->module_ref->name);
            }
        }
        while (it != tech->buffers_ports.end() && it->first == clk_conn.inst_ref->cell_ref->type) {
            PNR_LOG2("CLKT", "found an iobuf: '{}' by conn '{}'", clk_conn.inst_ref->cell_ref->type, clk_conn.makeName());
            for (auto& other_conn : clk_conn.inst_ref->conns) {
                if (other_conn.port_ref->name == it->second) {  // it's output of BUFG or IBUF
                    PNR_LOG2("CLKT", "recursing '{}'", other_conn.makeName());
                    recurseClockPeers(infos, other_conn, depth + 1, root);
                }
            }
            ++it;
        }
        // adding all data ports of reg-like instances clocked by this clock
        for (auto& other_conn : clk_conn.inst_ref->conns) {
            auto it = tech->clocked_ports.find(other_conn.inst_ref->cell_ref->type);
            bool clock_port = false;
            while (it != tech->clocked_ports.end() && it->first == other_conn.inst_ref->cell_ref->type) {
                if (other_conn.port_ref->name == it->second) {
                    clock_port = true;
                    break;
                }
                ++it;
            }
            if (it != tech->clocked_ports.end() && !clock_port && other_conn.port_ref->type == rtl::Port::PORT_IN) {
                PNR_LOG1("CLKT", "found conn '{}' of '{}' ('{}')", other_conn.makeName(), other_conn.inst_ref->makeName(), other_conn.inst_ref->cell_ref->type);
                infos->push_back( TimingInfo{.data_in = &other_conn} );
            }
        }
    }
    else
    for (auto* peer_ptr : clk_conn.peers) {  // it's CLK output, directly or from BUFG or from IBUF
        Referable<rtl::Conn>& peer = rtl::Conn::fromBase(*peer_ptr);
        PNR_LOG2("CLKT", "recursing '{}'", peer.makeName());
        recurseClockPeers(infos, peer, depth + 1, root);
    }
}

// follow all data ports of found clocked instances to build timing chains
bool Timings::recurseDataPeers(Referable<TimingPath>* path, int depth)
{
    rtl::Conn* curr = path->data_in;
    PNR_LOG2_("CLKT", depth, "tracing net '{}' from '{}', depth '{}'", curr->makeNetName(), curr->makeName(), depth);
    curr = curr->follow();
    if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
//            PNR_WARNING("cant trace conn '{}' of '{}' ('{}')", curr->makeName(), curr->inst_ref->cell_ref->name, curr->inst_ref->cell_ref->type);
        return false;
    }
    if (curr->inst_ref->locked) {
        PNR_WARNING("combinational loop was detected on '{}'('{}') while tracing delays for net '{}'", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type, curr->makeNetName());
        return false;
    }
    if (curr->inst_ref->timing.peer) {  // already calculated for this inst
        PNR_LOG2_("CLKT", depth, "already calculated");
        path->precalculated = curr->inst_ref->timing.peer;
        path->min_length = path->precalculated->min_length;
        path->max_length = path->precalculated->max_length;
        return true;
    }
    curr->inst_ref->locked = true;
    on_return autoexec([&curr]() {
        curr->inst_ref->locked = false;
    });
    PNR_LOG2_("CLKT", depth, "got '{}'('{}')", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type);
    path->data_output = curr;
    auto it1 = tech->clocked_ports.find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
    auto it2 = tech->buffers_ports.find(curr->inst_ref->cell_ref->type);
    if (it1 != tech->clocked_ports.end() || it2 != tech->buffers_ports.end()) {  // clocked or IOBUF
        path->min_length = 0;
        path->max_length = 0;
        return true;
    }
    int min_length = 1000000000;
    int max_length = -1;
    path->sub_paths.reserve(std::count_if(curr->inst_ref->conns.begin(), curr->inst_ref->conns.end(), [](auto& p) { return p.port_ref->type == rtl::Port::PORT_IN; }));
    for (auto& conn : curr->inst_ref->conns) {
        if (conn.port_ref->type == rtl::Port::PORT_IN) {
            path->sub_paths.emplace_back(TimingPath{.data_in = &conn});
            if (!recurseDataPeers(&path->sub_paths.back(), depth + 1)) {
                PNR_ASSERT(path->sub_paths.back().sub_paths.size() == 0, "pop not empty path");
                PNR_ASSERT(path->sub_paths.back().data_output == nullptr || path->sub_paths.back().data_output->inst_ref->timing.peer != &path->sub_paths.back(), "pop not empty path");
                PNR_ASSERT(path->sub_paths.back().precalculated == nullptr, "pop not empty path");
                path->sub_paths.pop_back();
            }
            else {
                if (path->sub_paths.back().min_length < min_length) {
                    min_length = path->sub_paths.back().min_length;
                }
                if (path->sub_paths.back().max_length > max_length) {
                    max_length = path->sub_paths.back().max_length;
                }
            }
        }
    }
    if (!path->sub_paths.size()) {
        return false;
    }
    path->min_length = min_length + 1;
    path->max_length = max_length + 1;
    PNR_LOG2_("CLKT", depth, "saving reference for '{}'('{}')", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type);
    curr->inst_ref->timing.set(path);
    return true;
}

// prepare timing calculation lists
void Timings::makeTimingsList(rtl::Design& design, clk::Clocks& clocks)
{
    PNR_LOG1("CLKT", "makeTimingsList, clocked_ports: {}, buffers_ports: {}", tech->clocked_ports, tech->buffers_ports);
/*    std::vector<rtl::Inst*> insts;
    std::vector<rtl::instFilter> filters;
    for (auto& cell_type : tech->clocked_ports) {  // can be duplicated filter
        filters.emplace_back(rtl::instFilter{});
        filters.back().blackbox = true;
        filters.back().cell_type = cell_type.first;
        PNR_LOG2("CLKT", "filter: {}", filters.back().format());
    }

    rtl::getInsts(&insts, filters, &design.top);
*/
    for (auto& clock : clocks.clocks_list) {
        auto& timings = clocked_inputs[&clock];
        recurseClockPeers(&timings, *clock.conn_ptr);  // find all clocked inputs for this clock

        for (auto& info : timings) {
            PNR_LOG2("CLKT", "checking conn '{}' of inst '{}' ('{}')", info.data_in->makeName(),
                info.data_in->inst_ref->makeName(), info.data_in->inst_ref->cell_ref->type);
            auto it = tech->clocked_ports.find(info.data_in->inst_ref->cell_ref->type);
            if (it == tech->clocked_ports.end()) {
                PNR_WARNING("unknown cell type: cant find cell type '{}' in clocked_port for inst '{}'\n",
                    info.data_in->inst_ref->cell_ref->type, info.data_in->inst_ref->makeName());
                continue;
            }
        }
    }

    PNR_LOG1("CLKT", "building timings tree");
    for (auto& clock : clocked_inputs) {
        PNR_LOG1("CLKT", "clock '{}' ('{}')", clock.first->name, clock.first->conn_name);
        for (auto& tinfo : clock.second) {
            tinfo.path.data_in = tinfo.data_in;
            recurseDataPeers(&tinfo.path);
        }
    }
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
        path.max_setup_time = 0;  // todo: add FD output delay here
        path.max_hold_time = 0;
        path.min_setup_time = 0;
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
            recurseTimings(tinfo.path);
        }
    }
}
