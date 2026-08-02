#pragma once

#include <cstddef>

namespace clk { struct Clocks; }
namespace fpga { struct Device; }
namespace technology { struct Tech; }

namespace pnr {

// Routes dedicated/high-fanout nets independently from ordinary fabric nets.
class RouteClocks
{
public:
    struct Stats
    {
        size_t clocks = 0;
        size_t buffers_placed = 0;
        size_t nets = 0;
        size_t sinks = 0;
        size_t routed = 0;
        size_t failed = 0;
        size_t graph_nodes = 0;
    };

    RouteClocks(technology::Tech& tech, fpga::Device& device);
    bool routeDesign(clk::Clocks& clocks);
    const Stats& stats() const { return stats_; }

private:
    technology::Tech& tech_;
    fpga::Device& device_;
    Stats stats_;
};

}
