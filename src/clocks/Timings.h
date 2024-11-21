#pragma once

#include "Design.h"
#include "getInsts.h"
#include "Inst.h"
#include "Conn.h"

#include <vector>
#include <string>

namespace clocks
{

struct Timings
{
    std::vector<rtl::Conn*> clocked_inputs;

    void makeTimingsList(const std::vector<std::string>& reg_types, const std::map<std::string,std::string>& clocked_ports)
    {
        std::vector<rtl::Inst*> insts;
        std::vector<rtl::instFilter> filters;
        for (auto& type : reg_types) {
            filters.emplace_back(rtl::instFilter{});
            filters.back().blackbox = true;
            filters.back().cell_type = type;
        }

        rtl::getInsts(&insts, filters, &rtl::Design::current().top);

        for (auto* inst : insts) {
            auto it = clocked_ports.find(inst->cell_ref->type);
            if (it == clocked_ports.end()) {
                PNR_WARNING("cant find clocked_port for cell type '{}'\n", inst->cell_ref->type);
                continue;
            }
            while (it != clocked_ports.end() && it->first == inst->cell_ref->type) {

                for (auto& conn : inst->conns) {
                    if (conn.port_ref->name == it->second) {
                        clocked_inputs.push_back(&conn);
                    }
                }
            }
        }
    }
};


}
