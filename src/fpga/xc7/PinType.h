#pragma once

namespace fpga {

struct PinType
{
    enum {
      PIN_IN,
      PIN_OUT,
      PIN_BIDIR,
      PIN_CLK,
      PIN_GCLK,
      PIN_LCLK,
    };
};

}
