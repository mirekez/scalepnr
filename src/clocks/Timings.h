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
#include <unordered_map>
#include <unordered_set>

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
        rtl::Conn* data_in = nullptr;
        Referable<TimingPath> path;
        double setup_limit = 0;
        double hold_limit = 0;
        bool constrained = true;
    };

    std::map<rtl::Clock*,std::vector<TimingInfo>> clocked_inputs;

    technology::Tech* tech = nullptr;

    void recurseClockPeers(std::vector<TimingInfo>* infos, Referable<rtl::Conn>& conn, int depth = 0, Referable<rtl::Conn>* root = 0);
    bool recurseDataPeers(Referable<TimingPath>* path, int depth = 0);

    void makeTimingsList(rtl::Design& design, Clocks& clocks);
    void recurseTimings(Referable<TimingPath>& path, int depth = 0);

    void calculateTimings();

private:
    // Scratch state for one capture-domain forest, discarded after building it.
    std::unordered_map<rtl::Conn*, TimingPath*> outputs;
    std::unordered_set<rtl::Conn*> visited_clock;
    std::unordered_set<rtl::Conn*> visited_data;
    Clocks* building_clocks = nullptr;
    rtl::Clock* capture_clock = nullptr;
};


}
