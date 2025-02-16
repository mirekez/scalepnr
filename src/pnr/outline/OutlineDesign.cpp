#include "OutlineDesign.h"
#include "Device.h"

using namespace pnr;

double OutlineDesign::recurseAllocation(int x, int y, MeshBox state[16], RegBunch& bunch, BunchLink* link, int depth)
{
//    if (!mark) {
//        mark = rtl::Inst::genMark();
//    }

    if (!link) {
//        if (bunch.reg->mark == travers_mark) {
//            return;  // already assigned to a box
//        }
//        bunch.reg->mark = travers_mark;

        state[y*4+x].size_luts += bunch.size_comb_own;
        bunch.reg->outline.x = x;
        bunch.reg->outline.y = y;
        bunch.reg->outline.set = true;

        double overal_distance = 0;
        bool need_distance = false;
        for (auto& link1 : bunch.uplinks) {
            if (link1.secondary) {
                need_distance = true;
                if (link1.conn->inst_ref->outline.set) {
                    double x_dist = (x - link1.conn->inst_ref->outline.x);
                    double y_dist = (y - link1.conn->inst_ref->outline.y);
                    double distance = (x_dist>=0?x_dist:-x_dist)+(y_dist>=0?y_dist:-y_dist);
                    if (link1.deficit > -0.5 || (bunch.clk_ref.peer && link1.delay > 0.5*bunch.clk_ref->period_ns)) {
                        distance *= 2;
                    }
                    if (link1.deficit > 0 || (bunch.clk_ref.peer && link1.delay > bunch.clk_ref->period_ns)) {
                        distance *= 3;
                    }
                    overal_distance += distance;
                    need_distance = false;
                }
            }
        }

        if (depth < 20) PNR_LOG2_("OUTL", depth, "recurseAllocation, bunch: {} ({}), x: {}, y: {}, distance: {}", bunch.reg->makeName(), bunch.reg->cell_ref->type, x, y, overal_distance);

        double min_distance = 0;
        for (auto& link1 : bunch.uplinks) {
            if (!link1.secondary) {
                double distance = recurseAllocation(x, y, state, bunch, &link1, depth + 1);
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }
        }

        bunch.reg->outline.set = false;

        return overal_distance + min_distance;
    }
    else {
        if (state[y*4+x].size_luts < cells_per_box) {  // same x, same y, fill box
            return recurseAllocation(x, y, state, *link->conn->inst_ref->bunch, nullptr, depth + 1);
        }
        double min_distance = 0;
        for (int dir=-1; dir <= 1; dir += 2) {
            if (x + dir >= 0 && x + dir < mesh_width && state[y*4+(x+dir)].size_luts < cells_per_box) {
                double distance = recurseAllocation(x + dir, y, state, *link->conn->inst_ref->bunch, nullptr, depth + 1);
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }
        }
        for (int dir=-1; dir <= 1; dir += 2) {
            if (y + dir >= 0 && y + dir < mesh_height && state[(y+dir)*4+x].size_luts < cells_per_box) {
                double distance = recurseAllocation(x, y + dir, state, *link->conn->inst_ref->bunch, nullptr, depth + 1);
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }
        }
        if (depth < 20) PNR_LOG2_("OUTL", depth, "recurseAllocation, x: {}, y: {}, link: {}, min_distance: {}, size_luts: {}", x, y, link->conn->makeName(), min_distance, state[y*4+x].size_luts);
        return min_distance;
    }
}

void OutlineDesign::optimizeOutline(std::list<Referable<RegBunch>>& bunch_list)
{
    auto& fpga = fpga::Device::current();

    int total_regs = 0;
    int total_luts = 0;

    for (auto& bunch : bunch_list) {
        total_regs += bunch.size_regs;
        total_luts += bunch.size_comb;  // need size of CARRY, MUX, SRL?   // then think about BRAM, LRAM, DSP
    }

    cells_per_box = total_luts / (mesh_width*mesh_height);

    PNR_LOG1("OUTL", "optimizeOutline, total_regs: {}, total_comb: {}, fpga - size_width: {}, size_height: {}, cnt_regs: {}, cnt_luts: {}, cells_per_box: {}",
        total_regs, total_luts, fpga.size_width, fpga.size_height, fpga.cnt_regs, fpga.cnt_luts, cells_per_box);


    for (auto& bunch : bunch_list) {
        for (int x=0; x < mesh_width; ++x) {
            for (int y=0; y < mesh_height; ++y) {
                PNR_LOG2("OUTL", "bunch: {}, x: {}, y: {}", bunch.reg->makeName(), x, y);
                MeshBox state[mesh_width*mesh_height];
                recurseAllocation(x, y, state, bunch);
            }
        }
    }
}
