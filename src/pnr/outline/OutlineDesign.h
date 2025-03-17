#pragma once

#include "Design.h"
#include "RegBunch.h"
#include "TileSet.h"
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

//struct State
//{
//    MeshBox boxes[SIZE*SIZE];
//    MeshBox total;
//    int dir;
//};

/*struct Node
{
    struct State state;
    double overall_distance;
    RegBunch* bunch;
    int parent_i;
    int x;
    int y;
    int dir = 0;
    bool branch;
    bool first_branch;
};
*/
struct OutlineDesign
{
    static constexpr const int mesh_width = 10;
    static constexpr const int mesh_height = 10;
    // must have
    double combs_per_box = 0;

    MeshBox boxes[mesh_height][mesh_width];

    void recurseStatsDesign(RegBunch& bunch, int depth = 0);

    uint64_t travers_mark = 0;
//    State state;
//    int level;
//    std::vector<Node> tree;
//    RegBunch* prev = nullptr;
    double avg_comb_in_bunch = 0;

    void attractBunch(RegBunch& bunch, int x, int y, int depth = 0, RegBunch* exclude = 0);
    uint64_t recurseSecondaryLinks(RegBunch& bunch, int depth = 0);
    void recurseRadialAllocation(RegBunch& bunch, int x, int y, int depth = 0);

    void optimizeOutline(std::list<Referable<RegBunch>>& bunch_list);

    void recursePrintDesign(std::list<Referable<RegBunch>>& bunch_list, int i, int depth = 0);
    png_draw image;
};


}
