#pragma once

#include "Design.h"
#include "Clocks.h"
#include "Inst.h"
#include "Conn.h"
#include "Timing.h"
#include "referable.h"

#include <vector>
#include <string>

namespace clocks
{

struct Timings
{
    struct TimingInfo
    {
        rtl::Conn* data_input;
        Referable<rtl::Timing> path;
    };

    std::map<clocks::Clock*,std::vector<TimingInfo>> clocked_inputs;

    void recurseClockPeers(std::vector<TimingInfo>* infos, Referable<rtl::Conn>& conn,
        const std::multimap<std::string,std::string>& clocked_ports,
        const std::multimap<std::string,std::string>& iobufs_ports,
        int depth = 0, Referable<rtl::Conn>* root = 0);

    void makeTimingsList(const std::multimap<std::string,std::string>& clocked_ports, const std::multimap<std::string,std::string>& iobufs_ports);

    bool recurseDataPeers(Referable<rtl::Timing>* path,
        const std::multimap<std::string,std::string>& clocked_ports,
        const std::multimap<std::string,std::string>& iobufs_ports, int depth = 0);

    void calculateTimings(const std::multimap<std::string,std::string>& clocked_ports, const std::multimap<std::string,std::string>& iobufs_ports);
};


}