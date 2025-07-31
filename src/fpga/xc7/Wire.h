#pragma once

#include "Inst.h"
#include "debug.h"
#include "referable.h"
#include "WireType.h"

#include <vector>

namespace fpga {

struct Wire
{
    // must have
    Coord coord;
    enum {
      WIRE_NULL,
      WIRE_MESH,
      WIRE_CLB,
      WIRE_VCC,
      WIRE_GND,
    } type = WIRE_NULL;

    const std::string makeName() const
    {
        return std::format("INT_?_X{}Y{}", coord.x, coord.y);
    }

    void assign(rtl::Net* net);
};


}
