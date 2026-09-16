#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

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
        size_t tile_visit_rejects = 0;
    };

    struct Failure
    {
        std::string net_name;
        int x = -1;
        int y = -1;
        uint8_t node_type = 0;
        int node = -1;

        explicit operator bool() const { return !net_name.empty(); }
    };

    RouteClocks(technology::Tech& tech, fpga::Device& device);
    bool routeDesign(clk::Clocks& clocks);
    const Stats& stats() const { return stats_; }
    const Failure& failure() const { return failure_; }

private:
    technology::Tech& tech_;
    fpga::Device& device_;
    Stats stats_;
    Failure failure_;
};

}
