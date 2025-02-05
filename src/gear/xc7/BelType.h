#pragma once

#include <string>

namespace gear {

struct BelType
{
    std::string name;
    enum {
        NONE,
//        SKIP,
//        MUX,
        LUT,
        LUTRAM,
        FF,
        CARRY
/*        CLKINV,
        DMUX,
        ZHOLD_DELAY,
        IDELAY,
        MISR,
        INV,
        ININV,
        IFF,
        TFF*/
    } type;
//    size_t inputs = 2;
//    enum {
//        SIMPLE,
//        SHARE_PREV_INPUTS
//    } input_type = SIMPLE;
//    size_t outputs = 1;
};


}
