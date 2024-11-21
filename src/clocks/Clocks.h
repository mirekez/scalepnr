#pragma once

#include "Clock.h"
#include "Design.h"
#include "getInsts.h"
#include "referable.h"
#include "debug.h"

#include <list>

namespace clocks
{

struct Clocks
{
    std::list<Clock> clocks;

    bool addClocks(rtl::Design& rtl, std::string& name, std::string port_name, double period_ns, int duty)
    {
        for (auto& clock : clocks) {
            if (clock.name == name || port_name == clock.conn_name) {
                return false;
            }
        }

        std::vector<rtl::Inst*> insts;
        rtl::instFilter filter;
        filter.partial = false;
        filter.port_name = port_name;
        rtl::getInsts(&insts, std::move(filter), &rtl::Design::current().top);

        for (auto* inst : insts) {
            for (auto& conn : inst->conns) {
                auto iname = inst->makeName();
                std::string name = iname + (iname.length()?".":"") + conn.port_ref->name;
                if (name == port_name) {
                    clocks.push_back( Clock{.name = name, .conn_ptr = &conn, .conn_name = port_name, .period_ns = period_ns, .duty = duty} );
                    return true;
                }
            }
        }
        return false;
    }

    bool getClocks(std::vector<Clock*>* clocks, std::string name, bool partial_name = true);

    static Clocks& current();
};


}
