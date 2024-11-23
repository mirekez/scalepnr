#pragma once

#include "Clock.h"
#include "Design.h"
#include "getConns.h"
#include "referable.h"
#include "debug.h"

#include <list>

namespace clocks
{

struct Clocks
{
    std::list<Clock> clocks_list;

    bool addClocks(rtl::Design& rtl, const std::string& clk_name, const std::string& port_name, double period_ns, int duty)
    {
        for (auto& clock : clocks_list) {
            if (clock.name == clk_name || port_name == clock.conn_name) {
                PNR_WARNING("clock '{}' already exists\n", clk_name);
                return false;
            }
        }

        std::vector<Referable<rtl::Conn>*> conns;
        rtl::connFilter filter;
        filter.partial = false;
        filter.port_name = port_name;
        rtl::getConns(&conns, std::move(filter), &rtl::Design::current().top);

        bool found = false;
        for (auto* conn : conns) {
            auto name = conn->makeName();
            if (name == port_name) {
                clocks_list.push_back( Clock{.name = clk_name, .conn_ptr = conn, .conn_name = port_name, .period_ns = period_ns, .duty = duty} );
                std::print("\ncreated clock '{}' for port '{}'", clk_name, name);
                return true;
            }
        }
        if (!found) {
            PNR_WARNING("cant find port '{}' for clock '{}'", port_name, clk_name);
        }
        return false;
    }

    void getClocks(std::vector<Clock*>* clocks, const std::string& name, bool partial_name = true);

    static Clocks& current();
};


}
