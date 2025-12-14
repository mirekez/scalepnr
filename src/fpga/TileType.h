#pragma once

#include <vector>

#include "Types.h"
#include "Pin.h"

namespace fpga {

struct TileType
{
    // must have
    std::string name;
    size_t num; //?
    int type;

    // optional
};

}
