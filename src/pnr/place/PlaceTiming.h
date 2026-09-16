#pragma once

#include "Timings.h"

#include <cstddef>
#include <memory>
#include <set>
#include <unordered_map>
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
    // Stable until the timing forest is rebuilt. Used to reconsider every
    // combinational input when a placement update changes the critical path.
    clk::TimingPath* timing_path = nullptr;
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
    std::unordered_map<const rtl::Inst*,
        std::unordered_map<const rtl::Inst*, double>> placement_net_weights;

    void preparePlacementGuide(clk::Timings& timings);
    double placementNetWeight(const rtl::Inst& inst,
                              const rtl::Inst& peer) const;
    PlaceTimingAnalysis analyze(clk::Timings& timings);
    // Fast local correction for an already selected critical path. This only
    // updates its placed wire delays, arrival, and setup slack; it does not
    // traverse the timing forest or search for a different critical path.
    void correctSetupTiming(PlaceTimingEndpoint& endpoint) const;
    void evaluateSetupTiming(const std::vector<PlaceTimingEndpoint*>& endpoints);
    std::vector<rtl::Inst*> setupDependencies(const PlaceTimingEndpoint& endpoint) const;
    double estimateWireDelay(const rtl::Conn& sink_input,
                             const rtl::Conn& driver_output) const;
    double estimateWireDelay(const rtl::Conn& sink_input,
                             const rtl::Conn& driver_output,
                             size_t fanout) const;
};

// Reusable exact evaluator for many placement trials over one timing forest.
// Connectivity, intrinsic delays and the forest must remain unchanged during
// its lifetime. Coordinates are read afresh on every evaluate() call; all
// combinational inputs are reconsidered, not just the previous critical path.
class PlaceTimingPrepared
{
    struct Impl;
    std::unique_ptr<Impl> impl;
public:
    explicit PlaceTimingPrepared(PlaceTiming& owner);
    ~PlaceTimingPrepared();
    void evaluate(const std::vector<PlaceTimingEndpoint*>& endpoints);
};

// Exact setup updates within the affected timing cones. Connectivity and the
// timing forest must remain unchanged for this object's lifetime. Force vectors
// and whole-graph work counters are refreshed only by a full analyze().
struct PlaceTimingIncremental
{
    struct Snapshot {
        size_t index;
        PlaceTimingEndpoint endpoint;
    };
    using Transaction = std::vector<Snapshot>;

    PlaceTiming& owner;
    PlaceTimingAnalysis& analysis;
    std::unordered_map<const rtl::Inst*, std::vector<size_t>> endpoints_by_cell;
    std::multiset<double> slacks;

    PlaceTimingIncremental(PlaceTiming& owner, PlaceTimingAnalysis& analysis);
    Transaction update(const std::vector<rtl::Inst*>& changed);
    // Refresh committed placement without allocating rollback snapshots.
    void updateForward(const std::vector<rtl::Inst*>& changed);
    void restore(Transaction&& transaction);
    double minimumSlack(const std::vector<rtl::Inst*>& cells) const;

protected:
    void refresh(const std::vector<rtl::Inst*>& changed, Transaction* transaction);
    void removeSlack(double slack);
    void addSlack(double slack);
};

// Forward-only local propagation on the existing TimingPath/Inst objects.
// No prepared graph or memoized evaluator. The forest and connectivity must
// remain stable, with only one direct updater active on a forest at a time.
class PlaceTimingLocal : public PlaceTimingIncremental
{
public:
    PlaceTimingLocal(PlaceTiming& owner, PlaceTimingAnalysis& analysis,
                     bool enabled = true);
    ~PlaceTimingLocal();
    PlaceTimingLocal(const PlaceTimingLocal&) = delete;
    PlaceTimingLocal& operator=(const PlaceTimingLocal&) = delete;
    void updateForward(const std::vector<rtl::Inst*>& changed);
    size_t updated_wires = 0;
    size_t updated_outputs = 0;
    size_t updated_endpoints = 0;

private:
    // Sorting commits forward; inherited speculative/rollback operations would
    // leave the live object values inconsistent and are deliberately hidden.
    using PlaceTimingIncremental::update;
    using PlaceTimingIncremental::restore;
    bool enabled;
    uint64_t change = 0;
    std::vector<clk::TimingPath*> paths;
    std::vector<rtl::Inst*> linked_cells;
    void bind(clk::TimingPath& path);
    void initializeInput(clk::TimingPath& path);
    void initializeOutput(clk::TimingPath& path);
    void updateEndpoint(size_t index);
};

}
