#pragma once

#include <vector>

#include "Types.h"
#include "BelType.h"
#include "Pin.h"

namespace gear {

    struct TileType
    {
        std::string name;
        size_t num;
        std::vector<BelType> bells;
        std::vector<Pin> bels_inputs;
        std::vector<Pin> bels_outputs;
        std::vector<Pin> site_inputs;
        std::vector<Pin> site_outputs;


  };

}
