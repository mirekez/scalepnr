#pragma once

#include <print>

namespace rtl
{

struct Timing
{
    // must have
    rtl::Conn* data_input;
    int max_length = -1;
    int min_length = -1;
    int fanout = 0;
    double own_setup_time = 0;  // delay till data_output (even if it does not exist)
    double own_hold_time = 0;
    double max_setup_time = 0;
    double max_hold_time = 0;
    double min_setup_time = 0;
    double min_hold_time = 0;
    // optional
    rtl::Conn* data_output = nullptr;
    std::vector<Referable<Timing>> sub_paths;
    Ref<Timing> precalculated;
};


}
