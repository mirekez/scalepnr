#include "Timings.h"
#include "debug.h"
#include "getInsts.h"
#include "on_return.h"

using namespace clocks;

void Timings::recurseClockPeers(std::vector<TimingInfo>* infos, Referable<rtl::Conn>& conn,
    const std::multimap<std::string,std::string>& clocked_ports,
    const std::multimap<std::string,std::string>& iobufs_ports,
    int depth, Referable<rtl::Conn>* root)
{
    if (root == 0) {
        root = &conn;
        PNR_LOG1("CLKT", "recurseClockPeers, root conn: '{}'", root->makeName());
    }
    if (conn.dependencies.size() == 0) {
        auto it = iobufs_ports.find(conn.inst_ref->cell_ref->type);
        if (it == iobufs_ports.end()) {
            PNR_LOG2("CLKT", "finished '{}'", conn.makeName());
            if (!conn.inst_ref->cell_ref->module_ref->is_blackbox) {
                PNR_WARNING("floating clock: got terminal conn '{}' ('{}') from clock input '{}', but it's not a is_blackbox ('{}')",
                    conn.makeName(), conn.inst_ref->cell_ref->type, root->makeName(), conn.inst_ref->cell_ref->module_ref->name);
            }
        }
        while (it != iobufs_ports.end() && it->first == conn.inst_ref->cell_ref->type) {
            PNR_LOG2("CLKT", "found an iobuf: '{}' by conn '{}'", conn.inst_ref->cell_ref->type, conn.makeName());
            for (auto& other_conn : conn.inst_ref->conns) {
                if (other_conn.port_ref->name == it->second) {  // internal
                    PNR_LOG2("CLKT", "recursing '{}'", other_conn.makeName());
                    recurseClockPeers(infos, other_conn, clocked_ports, iobufs_ports, depth + 1, root);
                }
            }
            ++it;
        }
        // adding port
        for (auto& other_conn : conn.inst_ref->conns) {
            auto it = clocked_ports.find(other_conn.inst_ref->cell_ref->type);
            bool clock_port = false;
            while (it != clocked_ports.end() && it->first == other_conn.inst_ref->cell_ref->type) {
                if (other_conn.port_ref->name == it->second) {
                    clock_port = true;
                    break;
                }
                ++it;
            }
            if (it != clocked_ports.end() && !clock_port && other_conn.port_ref->type == rtl::Port::PORT_IN) {
                PNR_LOG1("CLKT", "found conn '{}' of '{}' ('{}')", other_conn.makeName(), other_conn.inst_ref->makeName(), other_conn.inst_ref->cell_ref->type);
                infos->push_back( TimingInfo{.data_input = &other_conn} );
            }
        }
    }
    else
    for (auto& peer_ref : conn.dependencies) {
        auto& peer = static_cast<Referable<rtl::Conn>&>(*peer_ref);
        PNR_LOG2("CLKT", "recursing '{}'", peer.makeName());
        recurseClockPeers(infos, peer, clocked_ports, iobufs_ports, depth + 1, root);
    }
}

void Timings::makeTimingsList(const std::multimap<std::string,std::string>& clocked_ports, const std::multimap<std::string,std::string>& iobufs_ports)
{
    PNR_LOG1("CLKT", "makeTimingsList, clocked_ports: {}, iobufs_ports: {}", clocked_ports, iobufs_ports);
    std::vector<rtl::Inst*> insts;
    std::vector<rtl::instFilter> filters;
    for (auto& cell_type : clocked_ports) {  // can be duplicated filter
        filters.emplace_back(rtl::instFilter{});
        filters.back().blackbox = true;
        filters.back().cell_type = cell_type.first;
        PNR_LOG2("CLKT", "filter: {}", filters.back().format());
    }

    rtl::getInsts(&insts, filters, &rtl::Design::current().top);

    std::vector<Clock*> clocks;
    clocks::Clocks::current().getClocks(&clocks, "", true);

    for (auto* clock : clocks) {
        auto& vect = clocked_inputs[clock];
        recurseClockPeers(&vect, *clock->conn_ptr, clocked_ports,iobufs_ports);

        for (auto& info : vect) {
            PNR_LOG2("CLKT", "checking conn '{}' of inst '{}' ('{}')", info.data_input->makeName(),
                info.data_input->inst_ref->makeName(), info.data_input->inst_ref->cell_ref->type);
            auto it = clocked_ports.find(info.data_input->inst_ref->cell_ref->type);
            if (it == clocked_ports.end()) {
                PNR_WARNING("unknown cell type: cant find cell type '{}' in clocked_port for inst '{}'\n",
                    info.data_input->inst_ref->cell_ref->type, info.data_input->inst_ref->makeName());
                continue;
            }
        }
    }
}

bool Timings::recurseDataPeers(Referable<rtl::Timing>* path,
    const std::multimap<std::string,std::string>& clocked_ports,
    const std::multimap<std::string,std::string>& iobufs_ports, int depth)
{
    rtl::Conn* curr = path->data_input;
    PNR_LOG2("CLKT", "tracing net '{}' from '{}', depth '{}'", curr->makeNetName(), curr->makeName(), depth);
    for (rtl::Conn* conn = curr; conn; conn = conn->ref) {  // follow connection
        PNR_LOG3("CLKT", " <<'{}'('{}')", conn->makeName(), conn->inst_ref->cell_ref->type);
        curr = conn;
    }
    if (curr == path->data_input || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
//            PNR_WARNING("cant trace conn '{}' of '{}' ('{}')", curr->makeName(), curr->inst_ref->cell_ref->name, curr->inst_ref->cell_ref->type);
        path->min_length = 0;
        path->max_length = 0;
        return false;
    }
    if (curr->inst_ref->locked) {
        PNR_WARNING("combinational loop was detected on '{}'('{}') while tracing delays for net '{}'", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type, curr->makeNetName());
        path->min_length = 0;
        path->max_length = 0;
        curr->inst_ref->locked = false;
        return false;
    }
    if (curr->inst_ref->timing.ref) {  // already calculated for this inst
        PNR_LOG2("CLKT", "already calculated");
        path->precalculated.set(curr->inst_ref->timing.ref);
        path->min_length = path->precalculated->min_length;
        path->max_length = path->precalculated->max_length;
        return true;
    }
    curr->inst_ref->locked = true;
    on_return autoexec([&curr]() {
        curr->inst_ref->locked = false;
    });
    PNR_LOG2("CLKT", "got '{}'('{}')", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type);
    path->proxy = curr->inst_ref.ref;
    path->sub_paths.reserve(curr->inst_ref->conns.size());
    auto it1 = clocked_ports.find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
    auto it2 = iobufs_ports.find(curr->inst_ref->cell_ref->type);
    if (it1 != clocked_ports.end() || it2 != iobufs_ports.end()) {  // clocked or IOBUF
        path->min_length = 0;
        path->max_length = 0;
        return true;
    }
    int min_length = 1000000000;
    int max_length = -1;
    for (auto& conn : curr->inst_ref->conns) {  // comb
        if (conn.port_ref->type == rtl::Port::PORT_IN) {
            path->sub_paths.push_back(rtl::Timing{.data_input = &conn});
            if (!recurseDataPeers(&path->sub_paths.back(), clocked_ports, iobufs_ports, depth + 1)) {
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
    path->min_length = min_length + 1;
    path->max_length = max_length + 1;
    if (!path->sub_paths.size()) {
        return false;
    }
    PNR_LOG2("CLKT", "saving result for '{}'('{}')", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type);
    curr->inst_ref->timing.set(path);
    return true;
}

void Timings::calculateTimings(const std::multimap<std::string,std::string>& clocked_ports, const std::multimap<std::string,std::string>& iobufs_ports)
{
    PNR_LOG1("CLKT", "calculateTimings");
    for (auto& clock : clocked_inputs) {
        PNR_LOG2("CLKT", "clock '{}' ('{}')", clock.first->name, clock.first->conn_name);
        for (auto& tinfo : clock.second) {
            tinfo.path.data_input = tinfo.data_input;
            recurseDataPeers(&tinfo.path, clocked_ports, iobufs_ports);
        }
    }
}
