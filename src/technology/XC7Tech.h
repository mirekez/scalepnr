#pragma once

#include "Technology.h"
#include "Design.h"
#include "Timing.h"
#include "Timings.h"

struct XC7Tech: public Technology
{
    static std::multimap<std::string,std::string> clocked_ports;
    static std::multimap<std::string,std::string> buffers_ports;
    static std::map<std::string,std::pair<int,std::vector<double>>> comb_delays;  // for brief estimation

    clocks::Timings timings;

    void init();

    void recursivePrintTimingReport(rtl::Timing& path, unsigned limit = -1, int level = 0);
    void prepareTimingLists();
    void estimateTimings(unsigned limit_paths = 10, unsigned limit_rows = 3);

    static XC7Tech& current();

};
