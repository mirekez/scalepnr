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

struct RouteDesign
{
    tech::Tech* tech = nullptr;

    static constexpr const int mesh_width = 10;
    static constexpr const int mesh_height = 10;

    int fpga_width = 0;
    int fpga_height = 0;

    float aspect_x = 0;
    float aspect_y = 0;

    float image_zoom = 4;

    std::vector<Referable<fpga::Tile>>* tile_grid = nullptr;

    uint64_t travers_mark = 0;
    void recursivePackBunch(rtl::Inst& inst, RegBunch* bunch, int depth = 0);
    void routeDesign(std::list<Referable<RegBunch>>& bunch_list);
    void recurseDrawDesign(rtl::Inst& inst, RegBunch* bunch, int depth = 0);
    png_draw image;
};

}
