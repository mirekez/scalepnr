#pragma once

#include "Design.h"
#include "Inst.h"
#include "Conn.h"
#include "Clock.h"
#include "Clocks.h"
#include "TimingPath.h"
#include "referable.h"

#include <vector>
#include <string>

namespace technology
{
    struct Tech;
}

namespace clk
{

struct Clocks;

struct Timings
{
    struct TimingInfo
    {
        rtl::Conn* data_in;
        Referable<TimingPath> path;
        double setup_limit = 0;
        double hold_limit = 0;
    };

    std::map<rtl::Clock*,std::vector<TimingInfo>> clocked_inputs;

    technology::Tech* tech = nullptr;

    void recurseClockPeers(std::vector<TimingInfo>* infos, Referable<rtl::Conn>& conn, int depth = 0, Referable<rtl::Conn>* root = 0);
    bool recurseDataPeers(Referable<TimingPath>* path, int depth = 0);

    void makeTimingsList(rtl::Design& design, Clocks& clocks);
    void recurseTimings(Referable<TimingPath>& path, int depth = 0);

    void calculateTimings();
};


}
