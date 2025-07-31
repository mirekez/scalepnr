#pragma once

#include <vector>

#include "Types.h"
#include "BelType.h"
#include "Pin.h"

namespace fpga {

struct WireType
{
    enum {
      WIRE_NULL,
      WIRE_SHORT,
      WIRE_MID,
      WIRE_LONG,
      WIRE_JUMP,
    } type = WIRE_NULL;
};


}
