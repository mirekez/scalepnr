#pragma once

#include <string>
#include <vector>

namespace rtl
{

struct Net
{
    // optional
    std::string name;
    std::vector<int> designators;
};


}
