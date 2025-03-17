#include "OutlineDesign.h"
#include "Device.h"

#include <math.h>

using namespace pnr;

void OutlineDesign::attractBunch(RegBunch& bunch, int x, int y, int depth, RegBunch* exclude)
{
//    if (depth > 3) return;
    PNR_LOG3_("OUTL", depth, "attractBunch, bunch: {} ({}), bunch.x: {}, bunch.y: {}, x: {}, y: {}", bunch.reg->makeName(), bunch.reg->cell_ref->type, bunch.x, bunch.y, x, y);

    if (bunch.parent && bunch.parent != exclude && ((int)round(bunch.parent->x) != (int)round(bunch.x) || (int)round(bunch.parent->y) != (int)round(bunch.y))) {
        attractBunch(*bunch.parent, x, y, depth+1, &bunch);
    }

    for (auto& subbunch : bunch.sub_bunches) {
//    for (auto& link : bunch.uplinks) {
//        auto& subbunch = *link.conn->inst_ref->bunch_ref.peer;
        if (&subbunch != exclude && ((int)round(subbunch.x) != (int)round(bunch.x) || (int)round(subbunch.y) != (int)round(bunch.y))) {
            attractBunch(subbunch, x, y, depth+1, &bunch);
        }
//    }
    }

    float step = travers_mark == 0 ? 0.1 : ( bunch.mark != travers_mark ? 0.05 : 0 );
    if (avg_comb_in_bunch != 0) step = 0.01;

    int rx = round(x);
    int ry = round(y);
    int rbx = round(bunch.x);
    int rby = round(bunch.y);

    if (rbx < rx - 1 || rbx > rx + 1 ) {
        bunch.x += (x > bunch.x ? step : -step);
        PNR_LOG3("OUTL", " bunch => x: {}", bunch.x);
    }

    if (rby < ry - 1 || rby > ry + 1 ) {
        bunch.y += (y > bunch.y ? step : -step);
        PNR_LOG3("OUTL", " bunch => y: {}", bunch.y);
    }
    bunch.mark = travers_mark;
}

uint64_t OutlineDesign::recurseSecondaryLinks(RegBunch& bunch, int depth)
{
    uint64_t diffs = 0;
    int secondary_uplinks = 0;
    int secondary_uplinks_placed = 0;
    uint64_t sum_distance = 0;

    for (auto& subbunch : bunch.sub_bunches) {
        sum_distance += recurseSecondaryLinks(subbunch, depth + 1);
    }

    if (avg_comb_in_bunch != 0 && bunch.size_comb_own > avg_comb_in_bunch/2) {
        attractBunch(bunch, mesh_width/2, mesh_height/2, 0, &bunch);
    }

    for (auto& link : bunch.uplinks) {
        if (link.secondary) {
            ++secondary_uplinks;
            if (link.conn->inst_ref->bunch_ref->x != -1) {
                ++secondary_uplinks_placed;
                int x_dist = bunch.x - link.conn->inst_ref->bunch_ref->x;
                int y_dist = bunch.y - link.conn->inst_ref->bunch_ref->y;
                if ((x_dist >= 0 ? x_dist : -x_dist) + (y_dist >= 0 ? y_dist : -y_dist) > 1) {
                    attractBunch(*link.conn->inst_ref->bunch_ref.peer, bunch.x, bunch.y, 0, &bunch);
                    attractBunch(bunch, link.conn->inst_ref->bunch_ref->x, link.conn->inst_ref->bunch_ref->y, 0, link.conn->inst_ref->bunch_ref.peer);
                    ++diffs;
                }

                x_dist = (bunch.x - link.conn->inst_ref->bunch_ref->x);
                y_dist = (bunch.y - link.conn->inst_ref->bunch_ref->y);
                uint64_t distance = (x_dist>=0?x_dist:-x_dist)+(y_dist>=0?y_dist:-y_dist);
                if (link.deficit > -0.5 || (bunch.clk_ref.peer && link.delay > 0.5*bunch.clk_ref->period_ns)) {
//                    distance *= 2;
                }
                if (link.deficit > 0 || (bunch.clk_ref.peer && link.delay > bunch.clk_ref->period_ns)) {
//                    distance *= 3;
                }
                sum_distance += distance > 1 ? distance : 0;
            }
        }
    }

    PNR_LOG2_("OUTL", depth, "recurseSecondaryLinks, bunch: {} ({}), sum_distance: {}, uplinks: {}, secondary: {}, placed: {}", bunch.reg->makeName(), bunch.reg->cell_ref->type,
        sum_distance, bunch.uplinks.size(), secondary_uplinks, secondary_uplinks_placed);
    return sum_distance;
}

void OutlineDesign::recurseRadialAllocation(RegBunch& bunch, int x, int y, int depth)
{
    PNR_LOG2_("OUTL", depth, "recurseRadialAllocation, bunch: {} ({}), x: {}, y: {}, size: {}", bunch.reg->makeName(), bunch.reg->cell_ref->type, x, y, bunch.size_comb);

    bunch.x = x;
    bunch.y = y;

    if (x == 0 && y != mesh_height-1) {
        ++y;
    }
    else
    if (y == mesh_height-1 && x != mesh_width-1) {
        ++x;
    }
    else
    if (x == mesh_width-1 && y != 0) {
        --y;
    }
    else
    if (y == 0 && x != 0) {
        --x;
    }

    for (auto& subbunch : bunch.sub_bunches) {
        recurseRadialAllocation(subbunch, x, y, depth + 1);
    }

}

void OutlineDesign::recurseStatsDesign(RegBunch& bunch, int depth)
{
    boxes[(int)round(bunch.y)][(int)round(bunch.x)].size_regs += bunch.size_regs_own;
    boxes[(int)round(bunch.y)][(int)round(bunch.x)].size_luts += bunch.size_comb_own;
    boxes[(int)round(bunch.y)][(int)round(bunch.x)].bunches.push_back(&bunch);

    for (auto& link : bunch.uplinks) {
        if (link.secondary) {
        }
        else {
        }
    }

    for (auto& subbunch : bunch.sub_bunches) {
        recurseStatsDesign(subbunch, depth + 1);
    }
}

void OutlineDesign::recursePrintDesign(std::list<Referable<RegBunch>>& bunch_list, int i, int depth)
{
    if (depth == 0) {
        image.init(mesh_width*100, mesh_height*100);
        image.clear();
    }

    for (auto& bunch : bunch_list) {

        image.draw_space(bunch.x*100, bunch.y*100, 0, 0, 255, 255, 50);
        image.draw_space(bunch.x*100, bunch.y*100, 0, 255, 0, 255, bunch.size_comb_own);

        for (auto& link : bunch.uplinks) {
            if (link.secondary) {
                image.draw_line(link.conn->inst_ref->bunch_ref->x*100, link.conn->inst_ref->bunch_ref->y*100, bunch.x*100, bunch.y*100, 255, 0, 0, 255);
            }
            else {
                image.draw_line(link.conn->inst_ref->bunch_ref->x*100, link.conn->inst_ref->bunch_ref->y*100, bunch.x*100, bunch.y*100, 0, 0, 200, 255);
            }
        }

        recursePrintDesign(bunch.sub_bunches, i, depth + 1);
    }

    if (depth == 0) {
        image.write(std::string("outline_output-") + std::to_string(i) + ".png");
    }
}

void OutlineDesign::optimizeOutline(std::list<Referable<RegBunch>>& bunch_list)
{
//    set_pixel(image_data, width, 100, 100, 255, 0, 0, 255);
//    draw_line(image_data, width, 50, 50, 450, 50, 0, 0, 255, 255);   // Horizontal blue line

    auto& fpga = fpga::Device::current();

    int total_bunches = 0;
    int total_regs = 0;
    int total_comb = 0;

    for (auto& bunch : bunch_list) {
        total_bunches += bunch.size;
        total_regs += bunch.size_regs;
        total_comb += bunch.size_comb;  // need size of CARRY, MUX, SRL?   // then think about BRAM, LRAM, DSP
    }
    combs_per_box = /*total_comb*/fpga.cnt_luts / (mesh_width*mesh_height);


    for (auto& bunch : bunch_list) {
        recurseRadialAllocation(bunch, 0, 0);
    }

/*        for (auto& bunch : bunch_list) {
    for (int i=0; i < 10; ++i) {
            uint64_t sum_distance = recurseSecondaryLinks(bunch);
        std::print(std::cerr, "i: {}, sum_distance: {}\n", i, sum_distance);
    }
        }
*/
travers_mark = 0;
avg_comb_in_bunch = 0;
    for (int i=0; i < 300; ++i) {
        recursePrintDesign(bunch_list, i);

        if (i > 100) {
            for (size_t y=0; y < mesh_height; ++y) {
                for (size_t x=0; x < mesh_width; ++x) {
                    boxes[y][x].size_regs = 0;
                    boxes[y][x].size_luts = 0;
                    boxes[y][x].bunches.clear();
                }
            }

            for (auto& bunch : bunch_list) {
                recurseStatsDesign(bunch);
            }

            std::print("\n");
            std::print("{}\n", combs_per_box);
            for (size_t y=0; y < mesh_height; ++y) {
                for (size_t x=0; x < mesh_width; ++x) {
                    std::print("{:5d}", boxes[y][x].size_luts);
                }
                std::print("\n");
            }
            std::print("\n");

            size_t min_x;
            size_t min_y;
            int min_luts = 1000000000;
            for (int y=mesh_height/2; y >= 0; --y) {
                for (int x=mesh_width/2; x >= 0; --x) {
                    if (boxes[y][x].size_luts < min_luts) {
                        min_luts = boxes[y][x].size_luts;
                        min_y = y;
                        min_x = x;
                    }
                }
            }

            for (size_t y=0; y < mesh_height; ++y) {
                for (size_t x=0; x < mesh_width; ++x) {
                    if (boxes[y][x].size_luts > combs_per_box) {
                        int num = boxes[y][x].size_luts - combs_per_box;
                        for (auto* bunch : boxes[y][x].bunches) {
                            attractBunch(*bunch, min_x, min_y, 0, bunch);
                            num -= bunch->size_comb_own;
                            if (num < 0) break;
                        }
                    }
                }
            }
        }


        uint64_t sum_distance = 0;
        for (auto& bunch : bunch_list) {
if (i > 50) {
travers_mark = rtl::Inst::genMark();
}
if (i > 100) {
avg_comb_in_bunch = total_comb / total_bunches;
travers_mark = 0;
}
if (i > 150) {
travers_mark = 0;
avg_comb_in_bunch = 0;
}
            sum_distance += recurseSecondaryLinks(bunch);

            PNR_LOG2("OUTL", "fixing bunch: {} ({}), sum_distance: {}", bunch.reg->makeName(), bunch.reg->cell_ref->type, sum_distance);
        }
        std::print(std::cerr, "i: {}, sum_distance: {}\n", i, sum_distance);
    }
}
