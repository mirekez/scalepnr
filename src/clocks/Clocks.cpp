#include "Clocks.h"

using namespace clk;

void Clocks::getClocks(std::vector<rtl::Clock*>* clocks, const std::string& name, bool partial_name)
{
    PNR_LOG1("CLGC", "getClocks, name: '{}', partial_name: '{}'", name, partial_name);
    for (auto& clock : clocks_list) {
        PNR_LOG2("CLGC", "clock_name: '{}' (port '{}')", clock.name, clock.conn_name);
        if (name == clock.name || (partial_name && (name.length() == 0 || clock.name.find(name) != std::string::npos))) {
            PNR_LOG1("CLGC", "found_clock: '{}'", clock.name);
            clocks->push_back(&clock);
        }
    }
}

