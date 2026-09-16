#pragma once

#include "Design.h"
#include "RegBunch.h"
#include "Inst.h"
#include "Clocks.h"
#include "png_draw.h"

#include <vector>
#include <string>
#include <array>
#include <functional>
#include <unordered_map>

namespace technology
{
    struct Tech;
}

namespace fpga
{
    struct Pin;
    struct Tile;
}

namespace pnr
{

int packageSitePosition(const fpga::Tile& tile, const std::string& site);

struct MeshBox
{
    int size_regs = 0;
    int size_luts = 0;
    int size_mem = 0;
    std::vector<RegBunch*> bunches;
};

struct OutlineCapacityTrace
{
    rtl::Inst* inst = nullptr;
    fpga::Coord preferred{-1, -1};
    fpga::Coord selected{-1, -1};
    size_t assignment_order = 0;
    int preferred_occupancy = 0;
    int preferred_capacity = 0;
    int assigned_peers = 0;
    double preferred_peer_distance = 0;
    double selected_peer_distance = 0;
    double selected_score = 0;
    bool preferred_available = false;
    bool used_preferred_directly = false;
};

struct RadialAnchorGuide
{
    double x_sum = 0;
    double y_sum = 0;
    double depth_sum = 0;
    size_t count = 0;
};

struct OutlineDesign
{
    technology::Tech* tech = nullptr;
    static constexpr const int mesh_width = 10;
    static constexpr const int mesh_height = 10;
    int fpga_width;
    int fpga_height;
    float aspect_x = 0;
    float aspect_y = 0;
    float step_x = 0;
    float step_y = 0;
    // must have
    double combs_per_box = 0;

    void placeIOBs(std::list<Referable<RegBunch>>& bunch_list, std::map<std::string,std::string>& assignments, int depth = 0);
    void placeInstIOBs(rtl::Inst& inst, std::map<std::string,std::string>& assignments, int depth = 0);

    MeshBox boxes[mesh_height][mesh_width];
    int *boxes1;


    uint64_t travers_mark = 0;
    double avg_comb_in_bunch = 0;
    int iteration_limit = 1;
    bool uniform_unanchored_allocation = false;
    size_t allocation_cursor = 0;
    int allocation_register_target = 0;
    int allocation_comb_target = 0;
    std::array<int, mesh_width*mesh_height> allocated_registers{};
    std::array<int, mesh_width*mesh_height> allocated_combs{};
    std::unordered_map<rtl::Inst*, std::vector<rtl::Inst*>> optimization_peers;
    std::unordered_map<rtl::Inst*, std::vector<rtl::Inst*>> optimization_sinks;
    std::unordered_map<rtl::Inst*, std::vector<rtl::Inst*>> optimization_drivers;
    std::vector<rtl::Inst*> optimization_order;
    std::vector<std::pair<rtl::Inst*, rtl::Inst*>> optimization_edges;
    std::unordered_map<RegBunch*, RadialAnchorGuide> radial_anchor_guides;
    // Physical cross-bunch COMB connections absent from Estimate's register
    // uplinks when traversal stops at logic already owned by another bunch.
    std::unordered_map<RegBunch*, std::vector<RegBunch*>> shared_comb_links;
    std::unordered_map<std::string, fpga::Pin*> package_pins;
    std::unordered_map<std::string, fpga::Tile*> package_tiles;
    bool record_capacity_history = false;
    bool legalize_capacity_in_outline = false;
    std::vector<OutlineCapacityTrace> capacity_history;
    size_t timing_attraction_roots = 0;
    size_t timing_attraction_zero_force_roots = 0;
    size_t timing_attraction_moved_cells = 0;
    std::function<void(const std::string&)> debug_snapshot;

    void preparePackageLookup();

    void attractBunch(RegBunch& bunch, int x, int y, int depth = 0,
                      RegBunch* exclude = 0, bool propagate = true);
    uint64_t recurseSecondaryLinks(RegBunch& bunch, int depth = 0);
    size_t prepareSharedCombLinks(std::list<Referable<RegBunch>>& bunch_list);
    void recurseStatsDesign(RegBunch& bunch, int depth = 0);
    RadialAnchorGuide prepareRadialAnchorGuides(RegBunch& bunch);
    void recurseRadialAllocation(RegBunch& bunch, float x, float y,
                                 int depth = 0);

    void optimizeOutline(std::list<Referable<RegBunch>>& bunch_list);

    void recurseInstAllocation(rtl::Inst& inst, RegBunch* bunch, int depth = 0);
    void recurseInstPrepare(rtl::Inst& inst, RegBunch* bunch, int depth = 0);
    void recurseOptimizeInsts(rtl::Inst& inst, RegBunch* bunch, int i, int depth = 0);
    size_t moveTimingAttractorsSimultaneously();
    bool isGravityAttractor(const rtl::Inst& inst) const;
    void attractInst(rtl::Inst& inst, RegBunch* bunch, float step, float x, float y, int i, rtl::Inst* exclude, int depth = 0);
    void legalizeOutlineCapacity();

    float image_zoom = 2;
    void recurseDrawOutline(std::list<Referable<RegBunch>>& bunch_list, int i, int depth = 0);
    void recurseDrawDesign(rtl::Inst& inst, RegBunch* bunch, int mode, int depth = 0);
    void recurseDumpDesign(rtl::Inst& inst, RegBunch* bunch, FILE* out, int depth = 0);
    png_draw image;
};


}
