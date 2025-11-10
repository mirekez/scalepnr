#pragma once

#include <vector>

#include "Types.h"
#include "ElemType.h"
#include "Pin.h"

namespace fpga {

struct TileType
{
    // must have
    std::string name;
    size_t num; //?
    int type;

    // optional
    std::vector<ElemType> Elems;
    std::vector<Pin> elemInputs;
    std::vector<Pin> elemOutputs;
};

}
