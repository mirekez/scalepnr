#pragma once

namespace rtl
{

struct Timing
{
    // must have
    rtl::Conn* data_input;
    int max_length = -1;
    int min_length = -1;
    double setup_time = 0;
    double hold_time = 0;
    double setup_limit = 0;
    double hold_limit = 0;
    // optional
    rtl::Inst* proxy = nullptr;
    std::vector<Referable<Timing>> sub_paths;
    Ref<Timing> precalculated;  // if this inst already was calculated
};


}
