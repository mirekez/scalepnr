#pragma once

#include "Inst.h"
#include "debug.h"
#include "referable.h"
#include "Pin.h"

#include <string>
#include <vector>

namespace fpga {

struct Wire
{
    enum Type {
      WIRE_CROSSBAR,
      WIRE_TILE_PIN,
    } type = WIRE_CROSSBAR;

    // must have
    Coord from;
    Coord to;

    // optional metadata for local crossbar node to tile resource pin hops
    int local = -1;
    int pos = -1;
    int jump = -1;
    int joint = -1;
    std::string port;
    std::string net_name;

    void assign(rtl::Net* net);
};


}
