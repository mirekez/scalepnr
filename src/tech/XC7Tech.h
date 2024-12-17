#pragma once

#include "Tech.h"
#include "Design.h"
#include "Timing.h"
#include "Timings.h"

struct XC7Tech: public Tech
{
    static std::multimap<std::string,std::string> clocked_ports;
    static std::multimap<std::string,std::string> buffers_ports;
    static std::map<std::string,std::pair<int,std::vector<double>>> comb_delays;  // for brief estimation

    void init();

    void recursivePrintTimingReport(clk::TimingPath& path, unsigned limit = -1, int level = 0);
    void prepareTimingLists();
    void estimateTimings(unsigned limit_paths = 10, unsigned limit_rows = 3);

    void open_design();

    static XC7Tech& current();

};
