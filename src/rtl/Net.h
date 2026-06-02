#pragma once

#include "referable.h"
#include "Port.h"

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace fpga {
struct Wire;
}

namespace rtl
{

struct Inst;

struct NetRouteBinding
{
    Inst* owner = nullptr;
    size_t route_index = std::numeric_limits<size_t>::max();
    Inst* from = nullptr;
    Inst* to = nullptr;
    std::string from_port;
    std::string to_port;
    std::string route_name;
};

struct Net
{
    // optional
    std::string name;
    std::vector<int> designators;

    Ref<fpga::Wire> wire;
    Ref<Port> src_port;
    Ref<Port> dst_port;
    bool void_net = false;
    std::vector<NetRouteBinding> routes;
    std::string makeName(size_t limit = 200)
    {
        return name;
    }
};


}
