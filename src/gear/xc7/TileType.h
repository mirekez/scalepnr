#pragma once

#include <vector>

#include "Types.h"
#include "BelType.h"
#include "Pin.h"

namespace gear {

struct TileType
{
    // must have
    std::string name;
    size_t num;
    // optional
    std::vector<BelType> bells;
    std::vector<Pin> bels_inputs;
    std::vector<Pin> bels_outputs;
    std::vector<Pin> site_inputs;
    std::vector<Pin> site_outputs;
};


}
