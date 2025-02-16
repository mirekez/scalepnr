#pragma once

#include "Design.h"
#include "RegBunch.h"
#include "TileSet.h"
#include "Inst.h"
#include "Tech.h"
#include "Clocks.h"

#include <vector>
#include <string>

namespace pnr
{

struct MeshBox
{
    int size_regs = 0;
    int size_luts = 0;
    int size_mem = 0;
};

struct OutlineDesign
{
    // must have
    const int mesh_width = 4;
    const int mesh_height = 4;
    double cells_per_box = 0;

    double recurseAllocation(int x, int y, MeshBox state[16], RegBunch& bunch, BunchLink* link = 0, int depth = 0);
    void optimizeOutline(std::list<Referable<RegBunch>>& bunch_list);

};


}
