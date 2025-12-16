#pragma once

#include "Inst.h"
#include "debug.h"
#include "referable.h"
#include "Pin.h"

#include <vector>

namespace fpga {

struct Wire
{
    // must have
    Coord from;
    Coord to;

    void assign(rtl::Net* net);
};


}
