#pragma once

#include "Clock.h"
#include "Design.h"
#include "getConns.h"
#include "referable.h"
#include "debug.h"

#include <list>

#define MAX_CLOCKS 16

namespace technology
{
    struct Tech;
}

namespace clk
{

struct Clocks
{
    std::vector<Referable<rtl::Clock>> clocks_list;

    technology::Tech* tech = nullptr;

    bool addClocks(rtl::Design& design, const std::string& clk_name, const std::string& port_name, double period_ns, int duty);
    void getClocks(std::vector<rtl::Clock*>* clocks, const std::string& name, bool partial_name = true);
    void findBufs(Referable<rtl::Conn>* clk_conn, rtl::Clock& clk);
};


}
