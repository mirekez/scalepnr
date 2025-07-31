#pragma once

#include "Design.h"
#include "RegBunch.h"
#include "Inst.h"
#include "Tech.h"
#include "Clocks.h"
#include "png_draw.h"

#include <vector>
#include <string>

namespace pnr
{

struct MeshBox
{
    int size_regs = 0;
    int size_luts = 0;
    int size_mem = 0;
    std::vector<RegBunch*> bunches;
};

struct OutlineDesign
{
    tech::Tech* tech = nullptr;
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

    MeshBox boxes[mesh_height][mesh_width];
    int *boxes1;


    uint64_t travers_mark = 0;
    double avg_comb_in_bunch = 0;

    void attractBunch(RegBunch& bunch, int x, int y, int depth = 0, RegBunch* exclude = 0);
    uint64_t recurseSecondaryLinks(RegBunch& bunch, int depth = 0);
    void recurseStatsDesign(RegBunch& bunch, int depth = 0);
    void recurseRadialAllocation(RegBunch& bunch, int x, int y, int depth = 0);

    void optimizeOutline(std::list<Referable<RegBunch>>& bunch_list);

    void recurseInstAllocation(rtl::Inst& inst, RegBunch* bunch, int depth = 0);
    void recurseInstPrepare(rtl::Inst& inst, RegBunch* bunch, int depth = 0);
    void recurseOptimizeInsts(rtl::Inst& inst, RegBunch* bunch, int i, int depth = 0);
    void attractInst(rtl::Inst& inst, RegBunch* bunch, float step, float x, float y, int i, rtl::Inst* exclude, int depth = 0);

    float image_zoom = 2;
    void recurseDrawOutline(std::list<Referable<RegBunch>>& bunch_list, int i, int depth = 0);
    void recurseDrawDesign(rtl::Inst& inst, RegBunch* bunch, int mode, int depth = 0);
    void recurseDumpDesign(rtl::Inst& inst, RegBunch* bunch, FILE* out, int depth = 0);
    png_draw image;
};


}
