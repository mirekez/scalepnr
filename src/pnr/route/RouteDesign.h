#pragma once

#include "Design.h"
#include "RegBunch.h"
#include "TileSet.h"
#include "Inst.h"
#include "Clocks.h"
#include "png_draw.h"
#include "Device.h"

#include <vector>
#include <string>

namespace technology
{
    struct Tech;
}

namespace pnr
{

struct RouteDesign
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

    uint64_t travers_mark = 0;
    void recursiveRouteBunch(rtl::Inst& inst, RegBunch* bunch, int depth = 0);
    void routeDesign(std::list<Referable<RegBunch>>& bunch_list);
    void recurseDrawDesign(rtl::Inst& inst, RegBunch* bunch, int depth = 0);
    bool routeNet(rtl::Inst& from, rtl::Inst& to);
    bool tryNext(Tile& from, Tile& to, int from_pos, int to_pos, Wire& wire, int depth = 0);

    png_draw image;
};

}
