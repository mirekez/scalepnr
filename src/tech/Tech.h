#pragma once

#include "debug.h"
#include "Design.h"
#include "Clocks.h"
#include "Timings.h"
#include "EstimateDesign.h"
#include "OutlineDesign.h"
#include "PlaceDesign.h"
#include "RouteDesign.h"

#include <vector>
#include <string>
#include <map>

namespace technology
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
                PNR_LOG4("CLKT", " '{}':{}/{}={}/{}:{:.3f}", type,
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

    int tile_lutscnt = 0;
    int tile_lutstype = 0;
    int tile_regs = 0;
    int tile_carry = 0;
    int tile_memcnt = 0;
    int tile_memtype = 0;
    std::map<std::string,std::pair<int,int>> fixed_conns;

    bool check_clocked(std::string& type, std::string& port)
    {
        auto it = clocked_ports.find(type);  // we support now only 100% clocked or 100% combinational BELs
        while (it != clocked_ports.end()) {
            if (it->second == port) {  // clock port // TODO: add support for 2-clock primitives
                return true;
            }
            ++it;
        }
        return false;
    }

    rtl::Design design;
    clk::Clocks clocks;
    clk::Timings timings;
    pnr::EstimateDesign estimate;
    pnr::OutlineDesign outline;
    pnr::PlaceDesign place;
    pnr::RouteDesign route;
    std::map<std::string,std::string> assignments;

    void init();

    void recursivePrintTimingReport(clk::TimingPath& path, unsigned limit = -1, int level = 0);
    void prepareTimingLists();
    void estimateTimings(unsigned limit_paths = 10, unsigned limit_rows = 3);

    void loadDesign(const std::string& filename, const std::string& top_module);
    void openDesign();
    void placeDesign();
    void routeDesign();
    void printDesign(std::string& inst_name, int limit);

    static Tech& current();
};

void readTechMap(std::string maptext, TechMap& map);


}
