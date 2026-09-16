#pragma once

#include "Conn.h"

namespace clk
{

struct TimingPath
{
    // Live placed timing, maintained directly while a local placement updater
    // owns this forest. These are the actual edge/output values, not a second
    // compiled timing graph. Links are cleared before the updater is destroyed.
    struct Placement {
        const void* updater = nullptr;
        double wire_ns = 0;
        double input_arrival_ns = 0;
        double output_arrival_ns = 0;
        TimingPath* parent = nullptr;
        TimingPath* critical = nullptr;
        std::vector<TimingPath*> consumers;
        std::vector<size_t> endpoints;
        size_t input_level = 0;
        size_t output_level = 0;
        uint64_t wire_changed = 0;
        uint64_t input_changed = 0;
        bool input_ready = false;
        bool output_ready = false;
        bool active = false;
        bool input_queued = false;
        bool output_queued = false;
    } placement;

    // must have
    rtl::Conn* data_in = nullptr;
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
    TimingPath* precalculated = nullptr;
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
