#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace technology { struct Tech; }
namespace rtl { struct Inst; }

#ifndef SCALEPNR_PNR_RUN_CELLS
#define SCALEPNR_PNR_RUN_CELLS 1000
#endif

namespace pnr {

inline constexpr size_t default_run_cells = SCALEPNR_PNR_RUN_CELLS;

// Owns whole-design selection and the place -> route -> freeze run boundary.
// Stage algorithms remain in their respective engines.
class Pnr {
public:
    struct Run {
        std::vector<rtl::Inst*> cells;
        double deficit = 0;
        size_t completed_cells = 0;
        size_t eligible_connections = 0;
        size_t deferred_connections = 0;
        double placement_seconds = 0;
        double routing_seconds = 0;
        double estimated_wns_ns = 0;
        size_t unplaced_timing_edges = 0;
        bool success = false;
    };

    explicit Pnr(technology::Tech& tech) : tech_(tech) {}
    size_t run_cells = default_run_cells;
    std::vector<Run> runs;
    std::string failure;
    // Optional diagnostics called only after validation and freezing.
    std::function<void(const Run&)> run_completed;

    // Requires whole-design Estimate results; does not change physical state.
    void plan();
    bool run();

private:
    technology::Tech& tech_;
};

} // namespace pnr
