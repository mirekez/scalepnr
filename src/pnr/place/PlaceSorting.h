#pragma once

#include "PlaceTiming.h"

#include <cstddef>
#include <vector>

namespace technology {
struct Tech;
}

namespace rtl {
struct Inst;
}

namespace pnr {

enum class PlaceSortingDirection {
    none,
    north,
    east,
    south,
    west,
};

struct PlaceSortingConfig {
    double deficite_slack_ns = -0.10;
    double maximum_runtime_seconds = 30.0;
    // Zero means repeat live DEFICITE traversals until no progress or timeout.
    size_t maximum_passes = 0;
    // Experimental whole-chain destination, confined to Sorting.
    bool chain_center = false;
    bool trace_chain_moves = false;
    // Select capacity-feasible distances before constructing a cascade.
    // Disable only to compare against the exhaustive reference search.
    bool capacity_guided_displacement = true;
};

struct PlaceSortingChain {
    // Unique placed cells in launch-to-capture order, including fixed anchors.
    std::vector<rtl::Inst*> cells;
    double x = 0;
    double y = 0;
};

struct PlaceSortingMove {
    rtl::Inst* cell = nullptr;
    rtl::Inst* peer = nullptr;
    // Direction of the evacuated Tile contents, not the selected cell's move.
    PlaceSortingDirection direction = PlaceSortingDirection::none;
    fpga::Coord from{-1, -1};
    fpga::Coord to{-1, -1};
    fpga::Coord free_tile{-1, -1};
    size_t requested_shift = 0;
    size_t shifted_cells = 0;
    double slack_before_ns = 0;
    double slack_after_ns = 0;
    rtl::Conn* setup_endpoint = nullptr;
    bool toward_chain_center = false;
    double center_x = 0;
    double center_y = 0;
};

struct PlaceSortingResult {
    PlaceTimingAnalysis before;
    PlaceTimingAnalysis after;
    size_t deficite_cells = 0;
    size_t endpoints_examined = 0;
    size_t endpoint_sides_examined = 0;
    size_t chain_cells_examined = 0;
    size_t direction_attempts = 0;
    size_t free_tiles_examined = 0;
    size_t shift_attempts = 0;
    size_t accepted_moves = 0;
    size_t rejected_timing = 0;
    size_t rejected_packing = 0;
    size_t skipped_fixed = 0;
    size_t skipped_no_direction = 0;
    size_t skipped_no_free_tile = 0;
    size_t shifted_cells = 0;
    bool timed_out = false;
    size_t timing_evaluations = 0;
    size_t packing_previews = 0;
    size_t displacement_tiles_examined = 0;
    size_t impossible_displacements = 0;
    double elapsed_ms = 0;
    std::vector<PlaceSortingMove> moves;
};

struct PlaceSorting {
    technology::Tech* tech = nullptr;
    PlaceSortingConfig config;

    static PlaceSortingDirection directionFor(
        fpga::Coord cell, fpga::Coord peer, fpga::Coord device_size,
        PlaceSortingDirection preferred = PlaceSortingDirection::north);
    static PlaceSortingDirection nextDirection(PlaceSortingDirection direction);
    // Cyclic N/E/S/W order, starting at preferred; no chip-edge ranking.
    // The selected cell moves in the opposite direction, toward its peer.
    static std::vector<PlaceSortingDirection> evacuationDirections(
        fpga::Coord cell, fpga::Coord peer, fpga::Coord device_size,
        PlaceSortingDirection preferred = PlaceSortingDirection::north);
    static fpga::Coord directionStep(PlaceSortingDirection direction);
    static PlaceSortingChain chainCenter(const PlaceTimingEndpoint& endpoint);
    size_t estimateShiftTiles(
        double slack_ns, PlaceSortingDirection direction) const;

    PlaceSortingResult run(clk::Timings& timings);
    PlaceSortingResult run(
        clk::Timings& timings, const std::vector<rtl::Inst*>& cells);
};

const char* placeSortingDirectionName(PlaceSortingDirection direction);

}
