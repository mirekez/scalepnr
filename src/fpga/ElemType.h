#pragma once

#include <string>

namespace fpga {

struct ElemType
{
    std::string name;
    enum {
        NONE,
        LUT,
        LUTRAM,
        FF,
        CARRY
    } type;
//    size_t inputs = 2;
//    enum {
//        SIMPLE,
//        SHARE_PREV_INPUTS
//    } input_type = SIMPLE;
//    size_t outputs = 1;
};


}
