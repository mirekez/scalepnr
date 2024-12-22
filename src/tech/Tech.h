#pragma once

#include "debug.h"

#include <vector>
#include <string>
#include <map>

namespace tech
{

struct CombDelays
{
    std::map<std::string,std::pair<int,std::vector<double>>> map;

    double getDelay(const std::string& type, int index_in, int index_out)
    {
        auto it = map.find(type);
        if (it != map.end()) {
            if (index_in < 0 || index_out < 0) {
                PNR_WARNING("internal error: port index is not initialized at '{}'", type);
            }
            unsigned index = index_out*it->second.first/*portcnt*/ + index_in;
            if (index < it->second.second.size()) {
                PNR_LOG3("CLKT", " '{}':{}/{}={}/{}:{:.3f}", type,
                    index_in, index_out, index, it->second.second.size(), it->second.second[index]);
                }
                return it->second.second[index];
            }

        return 0;
    }
};

struct Tech
{
    static CombDelays comb_delays;
    static std::multimap<std::string,std::string> clocked_ports;
    static std::multimap<std::string,std::string> buffers_ports;
};


}
