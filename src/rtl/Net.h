#pragma once

#include <string>
#include <vector>

namespace fpga {
struct Wire;
}

namespace rtl
{

struct Net
{
    // optional
    std::string name;
    std::vector<int> designators;

    Ref<fpga::Wire> wire;
    std::string makeName(size_t limit = 200)
    {
        return name;
    }
};


}
