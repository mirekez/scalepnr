#pragma once

#include "Technology.h"
#include "Design.h"
#include "Timing.h"
#include "Timings.h"

struct XC7Tech: public Technology
{
    static std::multimap<std::string,std::string> clocked_ports;
    static std::multimap<std::string,std::string> buffers_ports;

    clocks::Timings timings;

    void init();

    bool estimateTimings(rtl::Design& design);
    void recursivePrintTimingReport(rtl::Timing& path, int level = 0);
    void prepareTimingLists();

    static XC7Tech& current();

};
