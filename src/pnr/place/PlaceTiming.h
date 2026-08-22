#pragma once

#include "Timings.h"

#include <cstddef>
#include <vector>

namespace technology
{
struct Tech;
}

namespace pnr
{

struct PlaceTimingCalibration
{
    // Architecture-independent first-order wire-delay calibration. Device-specific
    // calibration can replace these values after loading a device database.
    double local_wire_ns = 0.010;
    double horizontal_ns_per_tile = 0.035;
    double vertical_ns_per_tile = 0.040;
    double bend_ns = 0.005;
    double extra_fanout_ns = 0.002;
};

struct PlaceTimingEdge
{
    rtl::Conn* sink_input = nullptr;
    rtl::Conn* driver_output = nullptr;
    rtl::Inst* sink = nullptr;
    rtl::Inst* driver = nullptr;
    double wire_delay_ns = 0;
};

struct PlaceTimingEndpoint
{
    rtl::Clock* clock = nullptr;
    rtl::Conn* data_in = nullptr;
    double required_ns = 0;
    double arrival_ns = 0;
    double slack_ns = 0;
    std::vector<PlaceTimingEdge> critical_edges;
};

struct PlaceTimingForce
{
    rtl::Inst* inst = nullptr;
    rtl::Inst* strongest_peer = nullptr;
    double x = 0;
    double y = 0;
    double weight = 0;
    size_t violated_paths = 0;
};

struct PlaceTimingAnalysis
{
    size_t endpoints = 0;
    size_t evaluated_nodes = 0;
    size_t evaluated_edges = 0;
    size_t unplaced_edges = 0;
    size_t violated_endpoints = 0;
    double worst_slack_ns = 0;
    double total_negative_slack_ns = 0;
    double total_wire_delay_ns = 0;
    double elapsed_ms = 0;
    std::vector<PlaceTimingEndpoint> endpoint_details;
    std::vector<PlaceTimingForce> forces;
};

struct PlaceTimingRefinement
{
    PlaceTimingAnalysis before;
    PlaceTimingAnalysis after;
    size_t passes = 0;
    size_t attempted_cells = 0;
    size_t moved_cells = 0;
    size_t reverted_cells = 0;
    double elapsed_ms = 0;
};

struct PlaceTiming
{
    technology::Tech* tech = nullptr;
    PlaceTimingCalibration calibration;

    PlaceTimingAnalysis analyze(clk::Timings& timings);
    double estimateWireDelay(const rtl::Conn& sink_input,
                             const rtl::Conn& driver_output) const;
    double estimateWireDelay(const rtl::Conn& sink_input,
                             const rtl::Conn& driver_output,
                             size_t fanout) const;
};

}
