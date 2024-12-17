#pragma once

#include "Design.h"
#include "Clocks.h"
#include "Timings.h"
#include "Placing.h"

struct Tech
{

    rtl::Design design;
    clk::Clocks clocks;
    clk::Timings timings;
    pnr::Placing placing;

};
