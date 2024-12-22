#pragma once

#include "Conn.h"

namespace rtl
{

struct TimingPath
{
    // must have
    rtl::Conn* data_in;
    int max_length = -1;
    int min_length = -1;
    double own_setup_time = 0;  // delay till data_output (even if it does not exist)
    double own_hold_time = 0;
    double max_setup_time = 0;
    double max_hold_time = 0;
    double min_setup_time = 0;
    double min_hold_time = 0;
    // optional
    rtl::Conn* data_output = nullptr;
    std::vector<Referable<TimingPath>> sub_paths;
    TimingPath* precalculated;
};


}

//                              <--max_setup_time---------------------------------------> │
//                                                                                        │clk
//                              ┌──┐                        ┌──┐data_in          data_out┌▼─┐
//       <--own_setup_time----->│  │data_in                 │  │◄───────────────────────┐│┌┐│
//    │                         │  │◄──────────┐    data_out│  │◄───────────────────────┴┤│││
//clk │             ┌───────────┤  │◄─────────┐│┌───────────┤  │data_in    ┌─────────────┤└┘│
//   ┌▼─┐data_in    │   data_out└──┘data_in   │││           └──┘           │     data_out└──┘
//   │┌┐│◄──────────┘                         │└┼──────────────────────────┼────────────┐
//   │└┘│◄──────────┐           ┌──┐data_in   │ │           ┌──┐           │            │
//   └──┘data_in    │           │  │◄─────────┼─┘   data_out│  │◄──────────┘            │┌──┐
//                  └───────────┤  │          └─────────────┤  │data_in         data_out├┤┌┐│
//                      data_out│  │◄───────────────────────┤  │◄───────────────────────┘│└┘│
//                              └──┘data_in         data_out└──┘data_in                  └▲─┘
//                                                                                        │clk
//                                                                                        │
