#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace fpga { struct Device; }
namespace rtl { struct Inst; struct Net; }
namespace technology { struct Tech; }

namespace pnr {

// Discovers logical constant-one loads and gives them one stable physical-net
// identity. RouteDesign owns all path search, scheduling, and repair.
class RouteVCC
{
public:
    struct Sink
    {
        rtl::Inst* inst = nullptr;
        std::string port;
        std::string route_name;
    };

    struct PreparedRoutes
    {
        rtl::Inst* source = nullptr;
        rtl::Net* net = nullptr;
        std::vector<Sink> sinks;

        explicit operator bool() const
        {
            return source && net;
        }
    };

    struct Stats
    {
        size_t logical_sinks = 0;
        size_t prepared_sinks = 0;
    };

    RouteVCC(technology::Tech& tech, fpga::Device& device);
    PreparedRoutes prepareDesign();
    const Stats& stats() const { return stats_; }

private:
    technology::Tech& tech_;
    fpga::Device& device_;
    Stats stats_;
};

}
