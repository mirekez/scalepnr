#pragma once

#include "Design.h"
#include "Clocks.h"
#include "Inst.h"
#include "Conn.h"
#include "debug.h"
#include "getInsts.h"

#include <vector>
#include <string>

namespace clocks
{

struct Timings
{
    struct TimingPath
    {
        rtl::Conn* data_input;
        double setup_time = 0;
        double hold_time = 0;
        double setup_limit = 0;
        double hold_limit = 0;
        rtl::Inst* proxy;
        std::vector<TimingPath> sub_paths;
    };

    struct TimingInfo
    {
        rtl::Conn* data_input;
        TimingPath path;
    };
    std::map<clocks::Clock*,std::vector<TimingInfo>> clocked_inputs;

    void recurseClockPeers(std::vector<TimingInfo>* infos, Referable<rtl::Conn>& conn,
        const std::multimap<std::string,std::string>& clocked_ports,
        const std::multimap<std::string,std::string>& iobufs_ports,
        int depth = 0, Referable<rtl::Conn>* root = 0)
    {
        if (root == 0) {
            root = &conn;
            PNR_LOG1("CLKT", "recurseClockPeers, root conn: '{}'", root->makeName());
        }
        if (conn.dependencies.size() == 0) {
            auto it = iobufs_ports.find(conn.inst_ref->cell_ref->type);
            if (it == iobufs_ports.end()) {
                PNR_LOG2("CLKT", "finished '{}'", conn.makeName());
                if (!conn.inst_ref->cell_ref->module_ref->blackbox) {
                    PNR_WARNING("floating clock: got terminal conn '{}' ('{}') from clock input '{}', but it's not a blackbox ('{}')",
                        conn.makeName(), conn.inst_ref->cell_ref->type, root->makeName(), conn.inst_ref->cell_ref->module_ref->name);
                }
            }
            while (it != iobufs_ports.end() && it->first == conn.inst_ref->cell_ref->type) {
                PNR_LOG2("CLKT", "found a iobuf: '{}' by conn '{}'", conn.inst_ref->cell_ref->type, conn.makeName());
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
                while (it != clocked_ports.end() && it->first == other_conn.inst_ref->cell_ref->type) {
                    if (other_conn.port_ref->name == it->second) {
                        PNR_LOG1("CLKT", "found conn '{}' of '{}' ('{}')", other_conn.makeName(), other_conn.inst_ref->makeName(), other_conn.inst_ref->cell_ref->type);
                        infos->push_back( TimingInfo{.data_input = &other_conn} );
                    }
                    ++it;
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

    void makeTimingsList(const std::multimap<std::string,std::string>& clocked_ports, const std::multimap<std::string,std::string>& iobufs_ports)
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
                bool found = false;
                while (it != clocked_ports.end() && it->first == info.data_input->inst_ref->cell_ref->type) {
                    if (it->second == info.data_input->port_ref->name) {
                        found = true;
                    }
                    ++it;
                }
                if (!found) {
                    PNR_WARNING("unknown cell type: cant find port '{}' in clocked ports for type '{}' got by conn '{}'\n",
                        info.data_input->port_ref->name, info.data_input->inst_ref->cell_ref->type, info.data_input->inst_ref->makeName());
                }
            }
        }
    }

    void recurseDataPeers(TimingPath* path, const std::multimap<std::string,std::string>& clocked_ports, int depth = 0)
    {
        rtl::Conn* curr = path->data_input;
        PNR_LOG2("CLKT", "tracing net '{}' from '{}', depth '{}'", curr->makeNetName(), curr->makeName(), depth);
        for (rtl::Conn* conn = curr; conn; conn = conn->ref) {
            PNR_LOG3("CLKT", " >>'{}'('{}')", conn->makeName(), conn->inst_ref->cell_ref->type);
            curr = conn;
        }
        if (curr == path->data_input) {
//            PNR_WARNING("cant trace conn '{}' of '{}' ('{}')", curr->makeName(), curr->inst_ref->cell_ref->name, curr->inst_ref->cell_ref->type);
            return;
        }
        PNR_LOG2("CLKT", "got '{}'('{}')", curr->makeName(), curr->inst_ref->cell_ref->type);
        path->proxy = curr->inst_ref.ref;
        for (auto& conn : curr->inst_ref->conns) {
            auto it = clocked_ports.find(conn.inst_ref->cell_ref->type);
            bool found_clocked = false;
            while (it != clocked_ports.end() && it->first == conn.inst_ref->cell_ref->type) {
                found_clocked = true;
                ++it;
            }
            if (!found_clocked && conn.port_ref->type == rtl::Port::PORT_IN) {
                path->sub_paths.push_back(TimingPath{.data_input = &conn});
                recurseDataPeers(&path->sub_paths.back(), clocked_ports, depth + 1);
            }
        }
    }

    void calculateTimings(const std::multimap<std::string,std::string>& clocked_ports)
    {
        PNR_LOG1("CLKT", "calculateTimings");
        for (auto& clock : clocked_inputs) {
            PNR_LOG2("CLKT", "clock '{}' ('{}')", clock.first->name, clock.first->conn_name);
            for (auto& tinfo : clock.second) {
                tinfo.path.data_input = tinfo.data_input;
                recurseDataPeers(&tinfo.path, clocked_ports);
            }
        }
    }
};


}
