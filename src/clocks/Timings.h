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
    std::map<clocks::Clock*,std::vector<rtl::Conn*>> clocked_inputs;

    void recursePeers(std::vector<rtl::Conn*>* conns, Referable<rtl::Conn>& conn,
        const std::multimap<std::string,std::string>& clocked_ports,
        const std::multimap<std::string,std::string>& buffers_ports,
        int depth = 0, Referable<rtl::Conn>* root = 0)
    {
        if (root == 0) {
            root = &conn;
            PNR_LOG1("CLKT", "recursePeers, root conn: '{}'", root->makeName());
        }
        if (conn.dependencies.size() == 0) {
            auto it = buffers_ports.find(conn.inst_ref->cell_ref->type);
            if (it == buffers_ports.end()) {
                PNR_LOG2("CLKT", "finished '{}'", conn.makeName());
                if (!conn.inst_ref->cell_ref->module_ref->blackbox) {
                    PNR_WARNING("floating clock: got terminal conn '{}' ('{}') from clock input '{}', but it's not a blackbox ('{}')",
                        conn.makeName(), conn.inst_ref->cell_ref->type, root->makeName(), conn.inst_ref->cell_ref->module_ref->name);
                }
            }
            while (it != buffers_ports.end() && it->first == conn.inst_ref->cell_ref->type) {
                PNR_LOG2("CLKT", "found a buffer: '{}' by conn '{}'", conn.inst_ref->cell_ref->type, conn.makeName());
                for (auto& other_conn : conn.inst_ref->conns) {
                    if (other_conn.port_ref->name == it->second) {  // internal
                        PNR_LOG2("CLKT", "recursing '{}'", other_conn.makeName());
                        recursePeers(conns, other_conn, clocked_ports, buffers_ports, depth + 1, root);
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
                        conns->push_back(&other_conn);
                    }
                    ++it;
                }
            }
        }
        else
        for (auto& peer_ref : conn.dependencies) {
            auto& peer = static_cast<Referable<rtl::Conn>&>(*peer_ref);
            PNR_LOG2("CLKT", "recursing '{}'", peer.makeName());
            recursePeers(conns, peer, clocked_ports, buffers_ports, depth + 1, root);
        }
    }

    void makeTimingsList(const std::multimap<std::string,std::string>& clocked_ports, const std::multimap<std::string,std::string>& buffers_ports)
    {
        PNR_LOG1("CLKT", "makeTimingsList, clocked_ports: {}, buffers_ports: {}", clocked_ports, buffers_ports);

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
            recursePeers(&vect, *clock->conn_ptr, clocked_ports,buffers_ports);

            for (auto* conn : vect) {
                PNR_LOG2("CLKT", "checking conn '{}' of inst '{}' ('{}')", conn->makeName(), conn->inst_ref->makeName(), conn->inst_ref->cell_ref->type);
                auto it = clocked_ports.find(conn->inst_ref->cell_ref->type);
                if (it == clocked_ports.end()) {
                    PNR_WARNING("unknown cell type: cant find cell type '{}' in clocked_port for inst '{}'\n", conn->inst_ref->cell_ref->type, conn->inst_ref->makeName());
                    continue;
                }
                bool found = false;
                while (it != clocked_ports.end() && it->first == conn->inst_ref->cell_ref->type) {
                    if (it->second == conn->port_ref->name) {
                        found = true;
                    }
                    ++it;
                }
                if (!found) {
                    PNR_WARNING("unknown cell type: cant find port '{}' in clocked ports for type '{}' got by conn '{}'\n",
                        conn->port_ref->name, conn->inst_ref->cell_ref->type, conn->inst_ref->makeName());
                }
            }
        }

    }
};


}
