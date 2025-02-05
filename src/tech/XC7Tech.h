#pragma once

#include "Tech.h"
#include "Design.h"
#include "Timings.h"
#include "Placing.h"

using namespace tech;

struct XC7Tech: public Tech
{
    rtl::Design design;
    clk::Clocks clocks;
    clk::Timings timings;
    pnr::Placing placing;

    void init();

    void recursivePrintTimingReport(clk::TimingPath& path, unsigned limit = -1, int level = 0);
    void prepareTimingLists();
    void estimateTimings(unsigned limit_paths = 10, unsigned limit_rows = 3);

    void loadDesign(const std::string& filename, const std::string& top_module);
    void openDesign();
    void printDesign(std::string& inst_name, int limit);

    static XC7Tech& current();

};
