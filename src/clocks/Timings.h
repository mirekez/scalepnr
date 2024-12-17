#pragma once

#include "Design.h"
#include "Clocks.h"
#include "Inst.h"
#include "Conn.h"
#include "Timing.h"
#include "referable.h"

#include <vector>
#include <string>

namespace clk
{

struct Timings
{
    struct TimingInfo
    {
        rtl::Conn* data_in;
        Referable<TimingPath> path;
        double setup_limit = 0;
        double hold_limit = 0;
    };

    std::map<Clock*,std::vector<TimingInfo>> clocked_inputs;

    const std::multimap<std::string,std::string>* clocked_ports = nullptr;
    const std::multimap<std::string,std::string>* iobufs_ports = nullptr;

    void recurseClockPeers(std::vector<TimingInfo>* infos, Referable<rtl::Conn>& conn, int depth = 0, Referable<rtl::Conn>* root = 0);
    bool recurseDataPeers(Referable<TimingPath>* path, int depth = 0);

    void makeTimingsList(rtl::Design& design, clk::Clocks& clocks);
    void recurseTimings(Referable<TimingPath>& path, std::map<std::string,std::pair<int,std::vector<double>>>& comb_delays, int depth = 0);

    void calculateTimings(std::map<std::string,std::pair<int,std::vector<double>>>& comb_delays);
};


}
