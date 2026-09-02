#pragma once

#include "Design.h"
#include "Device.h"
#include "RegBunch.h"
#include "TileSet.h"
#include "Inst.h"
#include "Clocks.h"
#include "PlaceTiming.h"
#include "png_draw.h"

#include <vector>
#include <string>
#include <utility>
#include <chrono>
#include <array>
#include <unordered_map>
#include <unordered_set>

namespace technology
{
    struct Tech;
}

namespace pnr
{

enum class PlaceTimingMoveOutcome
{
    pending,
    accepted,
    anchor_blocked,
    anchor_reverted,
    objective_reverted,
};

enum class PlaceTimingMoveBlockReason
{
    none,
    immovable,
    zero_direction,
    boundary,
    invalid_tile,
    no_capacity,
};

struct PlaceTimingPlacementSnapshot
{
    rtl::Inst* inst = nullptr;
    Coord coord{-1, -1};
    int pos = -1;
};

struct PlaceTimingMoveTrace
{
    rtl::Inst* inst = nullptr;
    rtl::Inst* anchor = nullptr;
    rtl::Inst* strongest_peer = nullptr;
    Coord from{-1, -1};
    Coord to{-1, -1};
    Coord direction{0, 0};
    size_t run = 0;
    size_t pass = 0;
    double force_x = 0;
    double force_y = 0;
    double force_weight = 0;
    bool is_anchor = false;
    PlaceTimingMoveOutcome outcome = PlaceTimingMoveOutcome::pending;
    PlaceTimingMoveBlockReason block_reason = PlaceTimingMoveBlockReason::none;
};

struct InitialPlacementPeerTrace
{
    rtl::Inst* peer = nullptr;
    Coord target{-1, -1};
    double timing_weight = 0;
    bool already_placed = false;
};

struct InitialPlacementCandidateTrace
{
    Coord coord{-1, -1};
    int radius = -1;
    double timing_cost = 0;
    int occupied = 0;
    int placement_result = -2;
};

struct InitialPlacementTrace
{
    rtl::Inst* inst = nullptr;
    Coord origin{-1, -1};
    Coord selected{-1, -1};
    int radius = -1;
    std::vector<InitialPlacementPeerTrace> peers;
    std::vector<InitialPlacementCandidateTrace> candidates;
};

struct PlacePredictedMove
{
    rtl::Inst* inst = nullptr;
    Coord from{-1, -1};
    Coord to{-1, -1};
    Coord direction{0, 0};
    fpga::ElementType type = fpga::ELEMENT_LUT5;
    double external_force_x = 0;
    double external_force_y = 0;
    size_t external_peers = 0;
};

struct PlaceMovementFrame
{
    std::string filename;
    double progress = 0;
    std::vector<std::pair<double, double>> positions;
    std::vector<bool> active;
    std::vector<bool> deficite;
    std::vector<bool> proficite;
};

struct PlaceBunchReservation
{
    struct Placement
    {
        rtl::Inst* inst = nullptr;
        Coord coord{-1, -1};
        int pos = -1;
    };

    Coord minimum{-1, -1};
    Coord maximum{-1, -1};
    std::array<uint32_t, fpga::ELEMENT_TYPE_COUNT> demand{};
    std::vector<Placement> placements;
    size_t cells = 0;
};

struct PlacePreSmearResult
{
    size_t bunches = 0;
    size_t moved_bunches = 0;
    size_t moved_right = 0;
    size_t moved_down = 0;
    size_t right_first_bunches = 0;
    size_t down_first_bunches = 0;
    size_t reserved_cells = 0;
    size_t failed_bunches = 0;
    int maximum_shift = 0;
};

struct PlaceDesign
{
    technology::Tech* tech = nullptr;
    fpga::Device* fpga = nullptr;

    static constexpr const int mesh_width = 10;
    static constexpr const int mesh_height = 10;

    int fpga_width = 0;
    int fpga_height = 0;

    float aspect_x = 0;
    float aspect_y = 0;

    float image_zoom = 4;
    bool write_debug_images = true;
    std::string movement_png_prefix;
    int movement_png_frames = 0;
    rtl::Inst* movement_marker_a = nullptr;
    rtl::Inst* movement_marker_b = nullptr;
    std::vector<rtl::Inst*> movement_snapshot_cells;
    std::vector<PlaceMovementFrame> movement_snapshots;
    std::unordered_set<rtl::Inst*> movement_deficite_cells;
    std::unordered_set<rtl::Inst*> movement_proficite_cells;

    std::vector<Referable<fpga::Tile>>* tile_grid = nullptr;

    uint64_t travers_mark = 0;
    uint64_t place_calls = 0;
    uint64_t place_tile_trials = 0;
    uint64_t place_commits = 0;
    std::chrono::steady_clock::time_point place_started;
    std::chrono::steady_clock::time_point place_next_report;
    static constexpr int place_region_count = mesh_width * mesh_height;
    using CandidateList = std::vector<uint32_t>;
    std::array<std::array<CandidateList, place_region_count>, fpga::ELEMENT_TYPE_COUNT> place_candidates;
    std::array<std::array<size_t, place_region_count>, fpga::ELEMENT_TYPE_COUNT> place_candidate_cursor{};
    PlaceTiming place_timing;
    PlaceTimingRefinement timing_refinement;
    bool record_timing_history = false;
    std::vector<PlaceTimingPlacementSnapshot> placement_before_timing;
    std::vector<PlaceTimingMoveTrace> timing_move_history;
    std::vector<InitialPlacementTrace> initial_placement_history;
    std::unordered_map<RegBunch*, PlaceBunchReservation>
        bunch_reservations;
    std::vector<RegBunch*> bunch_reservation_order;
    size_t timing_refinement_runs = 0;
    void preparePlaceCandidates();
    int tryAddBySharedInput(rtl::Inst& inst, fpga::ElementType type, const Coord& origin);
    int tryAddSparseTile(rtl::Inst& inst, fpga::ElementType type, const Coord& origin);
    int tryAddNear(rtl::Inst& inst, fpga::ElementType type, const Coord& origin);
    int tryAddTimingAware(rtl::Inst& inst, fpga::ElementType type,
                          const Coord& origin);
    double timingPlacementCost(rtl::Inst& inst,
                               const Coord& candidate) const;
    std::vector<PlacePredictedMove> calculatePredictedDirections(
        const std::vector<rtl::Inst*>& cells,
        const std::unordered_map<rtl::Inst*, Coord>& predicted,
        size_t& oversubscribed_groups) const;
    size_t applyPredictedDisplacementSimultaneously(
        const std::vector<PlacePredictedMove>& moves,
        const std::vector<rtl::Inst*>* drawing_cells = nullptr,
        double displacement_scale = -1.0);
    PlacePreSmearResult preSmearBunches(
        const std::vector<rtl::Inst*>& cells);
    size_t commitPreSmearReservations();
    void smearOversubscribedCells(const std::vector<rtl::Inst*>& cells);
    void drawPlacementSnapshot(
        const std::vector<rtl::Inst*>& cells, const std::string& filename,
        const std::unordered_map<rtl::Inst*, std::pair<double, double>>*
            movement = nullptr,
        double progress = 0,
        const std::unordered_map<rtl::Inst*, std::pair<double, double>>*
            absolute_positions = nullptr,
        const std::unordered_set<rtl::Inst*>* deficite_cells = nullptr,
        const std::unordered_set<rtl::Inst*>* proficite_cells = nullptr);
    void captureMovementSnapshot(
        const std::vector<rtl::Inst*>& cells, const std::string& filename,
        const std::unordered_map<rtl::Inst*, std::pair<double, double>>*
            movement = nullptr,
        double progress = 0);
    std::string movementPngFilename(
        const std::string& stage_and_label) const;
    void redrawMovementSnapshotsWithMarkers();
    void recursivePackBunch(rtl::Inst& inst, RegBunch* bunch, int depth = 0);
    PlaceTimingRefinement refineTiming(clk::Timings& timings,
                                       size_t max_passes = 6,
                                       size_t max_anchor_cells_per_pass = 64);
    void placeDesign(std::list<Referable<RegBunch>>& bunch_list);
    void recurseDrawDesign(rtl::Inst& inst, RegBunch* bunch, int depth = 0);
    png_draw image;
};

}
