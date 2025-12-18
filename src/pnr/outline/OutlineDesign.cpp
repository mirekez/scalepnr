#include "OutlineDesign.h"
#include "Device.h"
#include "Tech.h"

#include <math.h>

using namespace pnr;

void OutlineDesign::placeIOBs(std::list<Referable<RegBunch>>& bunch_list, std::map<std::string,std::string>& assignments, int depth)
{
    for (auto& bunch : bunch_list) {
        PNR_LOG3_("OUTL", depth, "placeIOBs, bunch: {} ({})", bunch.reg->makeName(), bunch.reg->cell_ref->type);

        if (bunch.reg->cell_ref->type == "IBUF" || bunch.reg->cell_ref->type == "OBUF") {

            for (auto& conn : std::ranges::views::reverse(bunch.reg->conns)) {
                rtl::Conn* curr = &conn;
                if (bunch.reg->cell_ref->type == "IBUF" && curr->port_ref->type == rtl::Port::PORT_IN) {
                    if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // excluding clock ports
                        continue;
                    }

                    curr = curr->follow();
                    if (!curr /*|| !curr->inst_ref->cell_ref->module_ref->is_blackbox*/ || curr->port_ref->is_global) {  // after BUFs (can be something?)
                        continue;
                    }

                    auto port_name = curr->port_ref->name + (curr->port_ref->bitnum != -1 ? ("[" + std::to_string(curr->port_ref->bitnum) + "]") : "");

                    PNR_LOG2("OUTL", "placeIOBs, looking for assignments for '{}': '{}'", bunch.reg->makeName(), port_name);

                    auto it = assignments.find(port_name);
                    if (it != assignments.end()) {
                        PNR_LOG1("OUTL", "placeIOBs, found: '{}' for '{}', looking for pin...", it->second, port_name);

                        auto& device = fpga::Device::current();
                        auto& pins = device.pins;

                        for (auto& pin : pins) {
                            if (pin.name == it->second) {
                                PNR_LOG1("OUTL", "placeIOBs, found pin: '{}' coords ({},{})", pin.name, pin.pos.x, pin.pos.y)

                                auto it1 = device.x_to_grid.find(pin.pos.x==0?2:pin.pos.x-2);
                                auto it2 = device.y_to_grid.find(pin.pos.y);
                                if (it1 == device.x_to_grid.end()) {
                                    PNR_ERROR("cant find grid x pos for pin '{}' IBUF, pos ({},{})", pin.name, pin.pos.x, pin.pos.y);
                                    continue;
                                }
                                if (it2 == device.y_to_grid.end()) {
                                    PNR_ERROR("cant find grid y pos for pin '{}' IBUF, pos ({},{})", pin.name, pin.pos.x, pin.pos.y);
                                    continue;
                                }
                                int x = it1->second < 20 ? 0 : device.size_width;
                                bunch.reg->outline.fixed = true;
                                bunch.reg->outline.x = (float)x/device.size_width*mesh_width;
                                bunch.reg->outline.y = (float)it2->second/device.size_height*mesh_height;
                                bunch.fixed = true;
                                bunch.x = (float)x/device.size_width*mesh_width;
                                bunch.y = (float)it2->second/device.size_height*mesh_height;
                            }
                        }
                    }
                }
            }
        }

        placeIOBs(bunch.sub_bunches, assignments, depth + 1);
    }
}

void OutlineDesign::attractBunch(RegBunch& bunch, int x, int y, int depth, RegBunch* exclude)
{
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

    if (bunch.fixed) {
        return;
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

    if (!bunch.fixed) {
        bunch.x = (float)x + 0.5;
        bunch.y = (float)y + 0.5;

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

void OutlineDesign::recurseDrawOutline(std::list<Referable<RegBunch>>& bunch_list, int i, int depth)
{
    if (depth == 0) {
        image.init(mesh_width*100, mesh_height*100);
        image.clear();
    }

    for (auto& bunch : bunch_list) {

        image.draw_space(bunch.x*100, bunch.y*100, 0, 0, 255, 255, 50);
        image.draw_space(bunch.x*100, bunch.y*100, 0, 255, 0, 255, bunch.size_comb_own);

        for (auto& link : bunch.uplinks) {
            if (link.conn->inst_ref->outline.fixed || bunch.reg->outline.fixed) {
                image.draw_line(link.conn->inst_ref->bunch_ref->x*100, link.conn->inst_ref->bunch_ref->y*100, bunch.x*100, bunch.y*100, 200, 200, 200, 255);
            }
            else
            if (link.secondary) {
                image.draw_line(link.conn->inst_ref->bunch_ref->x*100, link.conn->inst_ref->bunch_ref->y*100, bunch.x*100, bunch.y*100, 255, 0, 0, 255);
            }
            else {
                image.draw_line(link.conn->inst_ref->bunch_ref->x*100, link.conn->inst_ref->bunch_ref->y*100, bunch.x*100, bunch.y*100, 0, 200, 200, 255);
            }
        }

        PNR_LOG2_("OUTL", depth, "recurseDrawOutline, bunch: {} ({}), x: {}, y: {}", bunch.reg->makeName(), bunch.reg->cell_ref->type, bunch.x, bunch.y);

        recurseDrawOutline(bunch.sub_bunches, i, depth + 1);
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
    combs_per_box = /*total_comb*/(float)fpga.cnt_luts / (mesh_width*mesh_height);

    fpga_width = fpga.size_width*2;
    fpga_height = fpga.size_height*2;
    aspect_x = (float)fpga_width/mesh_width;
    aspect_y = (float)fpga_height/mesh_height;
    step_x = (float)mesh_width/fpga_width;
    step_y = (float)mesh_height/fpga_height;

    boxes1 = new int[fpga_width*fpga_height];

    PNR_LOG1("OUTL", "optimizeOutline, fpga_width: {}, fpga_height: {}, aspect_x: {:.3f}, aspect_y: {:.3f}, step_x: {:.3f}, step_y: {:.3f}, total_regs: {}, total_comb: {}, total_bunches: {}, combs_per_box: {}",
        fpga_width, fpga_height, aspect_x, aspect_y, step_x, step_y, total_regs, total_comb, total_bunches, combs_per_box);

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
    for (int i=0; i < 50/*0*/; ++i) {
//std::print("---- {}\n", i);
//        recurseDrawOutline(bunch_list, i);

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
//        std::print(std::cerr, "i: {}, sum_distance: {}\n", i, sum_distance);
    }

    ////////////////////////////////////////// design

    travers_mark = rtl::Inst::genMark();
    for (auto& bunch : bunch_list) {
        recurseInstAllocation(*bunch.reg, &bunch);
    }

    travers_mark = rtl::Inst::genMark();
    for (auto& bunch : bunch_list) {
        recurseInstPrepare(*bunch.reg, &bunch);
    }

    for (int i=0; i < 50/*0*/; ++i) {
//        image.init(mesh_width*aspect_x*image_zoom, mesh_height*aspect_y*image_zoom);
//        image.clear();
//        travers_mark = rtl::Inst::genMark();
//        for (auto& bunch : bunch_list) {
//            recurseDrawDesign(*bunch.reg, &bunch, 1);
//        }
//        travers_mark = rtl::Inst::genMark();
//        for (auto& bunch : bunch_list) {
//            recurseDrawDesign(*bunch.reg, &bunch, 0);
//        }
//        image.write(std::string("design_output-") + std::to_string(i) + ".png");

//        travers_mark = rtl::Inst::genMark();
//        FILE* out = fopen((std::string("design_output-") + std::to_string(i) + ".txt").c_str(), "w");
//        for (auto& bunch : bunch_list) {
//            recurseDumpDesign(*bunch.reg, &bunch, out);
//        }
//        fclose(out);

        memset(boxes1, 0, fpga_width*fpga_height*sizeof(int));
        travers_mark = rtl::Inst::genMark();
        for (auto& bunch : bunch_list) {
//std::print("{} --- {} ({})\n", i, bunch.reg->makeName(), bunch.reg->cell_ref->type);fflush(stdout);
            recurseOptimizeInsts(*bunch.reg, &bunch, i);
        }
    }

    std::print("\n");
    for (int y=0; y < fpga_height; ++y) {
        for (int x=0; x < fpga_width; ++x) {
            std::print("{} ", boxes1[y*fpga_width + x]);
        }
        std::print("\n");
    }
    std::print("\n");
}

void OutlineDesign::recurseInstAllocation(rtl::Inst& inst, RegBunch* bunch, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

    PNR_LOG3_("OUTL", depth, "recurseInstAllocation, inst: {} ({}), x: {}, y: {}", inst.makeName(), inst.cell_ref->type, inst.bunch_ref->x + 0.5, inst.bunch_ref->y + 0.5);
    if (!inst.outline.fixed) {
        inst.outline.x = round(inst.bunch_ref->x) + 0.5;
        inst.outline.y = round(inst.bunch_ref->y) + 0.5;
    }

    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // excluding clock ports
                continue;
            }

            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
                continue;
            }

            rtl::Inst* peer = curr->inst_ref.peer;
            if (peer->bunch_ref.peer == inst.bunch_ref.peer) {
                if (peer->mark != travers_mark) {
                    recurseInstAllocation(*curr->inst_ref.peer, nullptr, depth + 1);
                }
            }
        }
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseInstAllocation(*subbunch.reg, &subbunch, depth + 1);

            bunch->x = round(bunch->x);
            bunch->y = round(bunch->y);
        }
    }
}

void OutlineDesign::recurseInstPrepare(rtl::Inst& inst, RegBunch* bunch, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // excluding clock ports
                continue;
            }

            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
                continue;
            }

            rtl::Inst* peer = curr->inst_ref.peer;
            if (peer->bunch_ref.peer != inst.bunch_ref.peer) {
                if (peer->outline.x > inst.outline.x + 0.5 && peer->outline.y > inst.outline.y + 0.5) {
                    inst.outline.x += 0.49;
                    inst.outline.y += 0.49;
                }
                else
                if (peer->outline.x > inst.outline.x + 0.5 && peer->outline.y < inst.outline.y - 0.5) {
                    inst.outline.x += 0.49;
                    inst.outline.y -= 0.49;
                }
                else
                if (peer->outline.x < inst.outline.x - 0.5 && peer->outline.y < inst.outline.y - 0.5) {
                    inst.outline.x -= 0.49;
                    inst.outline.y -= 0.49;
                }
                else
                if (peer->outline.x < inst.outline.x - 0.5 && peer->outline.y > inst.outline.y + 0.5) {
                    inst.outline.x -= 0.49;
                    inst.outline.y += 0.49;
                }
                else
                if (peer->outline.x > inst.outline.x + 0.5) {
                    inst.outline.x += 0.49;
                }
                else
                if (peer->outline.y > inst.outline.y + 0.5) {
                    inst.outline.y += 0.49;
                }
                else
                if (peer->outline.x < inst.outline.x - 0.5) {
                    inst.outline.x -= 0.49;
                }
                else
                if (peer->outline.y < inst.outline.y - 0.5) {
                    inst.outline.y -= 0.49;
                }
                PNR_LOG3_("OUTL", depth, "recurseInstPrepare, inst: {} ({}), x: {}, y: {}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y);
            }
            else {
                if (peer->mark != travers_mark) {
                    recurseInstPrepare(*peer, nullptr, depth + 1);
                }
            }
        }
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseInstPrepare(*subbunch.reg, &subbunch, depth + 1);
        }
    }
}

uint32_t eee = 0;

void OutlineDesign::recurseOptimizeInsts(rtl::Inst& inst, RegBunch* bunch, int i, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

    if (inst.outline.x < 0) {
        inst.outline.x = 0;
    }
    if (inst.outline.x > 9.99) {
        inst.outline.x = 9.95;
    }
    if (inst.outline.y < 0) {
        inst.outline.y = 0;
    }
    if (inst.outline.y > 9.99) {
        inst.outline.y = 9.95;
    }
//if (i<100 || (i>150 && i<200)) {
    int m = boxes1[(int)(inst.outline.y*aspect_y)*fpga_width + (int)(inst.outline.x*aspect_x)];
    ++eee;
    if (m > 1) {
        if ((eee%8==0 || eee%8==1) && inst.outline.x + step_x < inst.bunch_ref->x + 0.5 && boxes1[(int)(inst.outline.y*aspect_y)*fpga_width + (int)(inst.outline.x*aspect_x+1)] < m) {
            inst.outline.x += step_x;
        }
        if ((eee%8==2 || eee%8==3) && inst.outline.y + step_y < inst.bunch_ref->y + 0.5 && boxes1[(int)(inst.outline.y*aspect_y+1)*fpga_width + (int)(inst.outline.x*aspect_x)] < m) {
            inst.outline.y += step_y;
        }
        if ((eee%8==4 || eee%8==5) && inst.outline.x - step_x > inst.bunch_ref->x - 0.5 && boxes1[(int)(inst.outline.y*aspect_y)*fpga_width + (int)(inst.outline.x*aspect_x-1)] < m) {
            inst.outline.x -= step_x;
        }
        if ((eee%8==6 || eee%8==7) && inst.outline.y - step_y > inst.bunch_ref->y - 0.5 && boxes1[(int)(inst.outline.y*aspect_y-1)*fpga_width + (int)(inst.outline.x*aspect_x)] < m) {
            inst.outline.y -= step_y;
        }
    }
    if (inst.outline.x < 0) {
        inst.outline.x = 0;
    }
    if (inst.outline.x > 9.99) {
        inst.outline.x = 9.95;
    }
    if (inst.outline.y < 0) {
        inst.outline.y = 0;
    }
    if (inst.outline.y > 9.99) {
        inst.outline.y = 9.95;
    }
//}
    ++boxes1[(int)(inst.outline.y*aspect_y)*fpga_width + (int)(inst.outline.x*aspect_x)];

    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // excluding clock ports
                continue;
            }
            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
                continue;
            }

            rtl::Inst* peer = curr->inst_ref.peer;
//std::print("\n~~~{}", peer->cell_ref->name);


                if (peer->mark != travers_mark) {
////    inst.mark = travers_mark;

            if (peer->bunch_ref.peer != inst.bunch_ref.peer) {
//if ((i > 100 && i < 150) || i > 200) {
                attractInst(inst, bunch, step_x, peer->outline.x, peer->outline.y, i, peer, depth + 1);
////                attractInst(*peer, bunch, step_x, inst.outline.x, inst.outline.y, i, &inst, depth + 1);
//}
            }
            else {
//if ((i > 100 && i < 150) || i > 200) {
                attractInst(inst, bunch, step_x, peer->outline.x, peer->outline.y, i, peer, depth + 1);
////                attractInst(*peer, bunch, step_x, inst.outline.x, inst.outline.y, i, &inst, depth + 1);
//}
////                    peer->mark = travers_mark;
                    recurseOptimizeInsts(*peer, nullptr, depth + 1);
            }
                }
        }
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseOptimizeInsts(*subbunch.reg, &subbunch, i, depth + 1);
        }
    }
}

void OutlineDesign::attractInst(rtl::Inst& inst, RegBunch* bunch, float step, float x, float y, int i, rtl::Inst* exclude, int depth)
{
    PNR_LOG3_("OUTL", depth, "attractInst, inst: {} ({}), outline.x: {}, outline.y: {}, x: {}, y: {}, step: {}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y, x, y, step);

//if (inst.makeName() == "$abc$712025$auto$blifparse.cc:535:parse_blif$718286") {
//    std::print("\n!!!!!!!!!!!!!!!!!! attractInst, inst: {} ({}), outline.x: {}, outline.y: {}, x: {}, y: {}, step: {}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y, x, y, step);
//}
    PNR_ASSERT(inst.bunch_ref.peer != nullptr, "bunch ref of inst '{}' is zero", inst.makeName());

    if (!inst.outline.fixed)
    if ((i > 50 && boxes1[(int)(inst.outline.x + (x > inst.outline.x ? step : -step))*fpga_width + (int)(inst.outline.y + (y > inst.outline.y ? step : -step))] == 0)
        || (inst.outline.x + (x > inst.outline.x ? step : -step) < inst.bunch_ref->x + 0.5 && inst.outline.x + (x + inst.outline.x ? step : -step) > inst.bunch_ref->x - 0.5
        && inst.outline.y + (y > inst.outline.y ? step : -step) < inst.bunch_ref->y + 0.5 && inst.outline.y + (y + inst.outline.y ? step : -step) > inst.bunch_ref->y - 0.5)) {

        inst.outline.x += (x-step_x > inst.outline.x ? step : (x+step_x < inst.outline.x ? -step : 0));
        inst.outline.y += (y-step_y > inst.outline.y ? step : (y+step_y < inst.outline.y ? -step : 0));

//if (inst.makeName() == "$abc$712025$auto$blifparse.cc:535:parse_blif$718286") {
//    std::print("\n!!!!!!!!!!!!!!!!!! attractInst, x: {}, y: {}", inst.outline.x, inst.outline.y);
//}

        for (auto& conn : std::ranges::views::reverse(inst.conns)) {
            rtl::Conn* curr = &conn;
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // excluding clock ports
                continue;
            }

            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
                continue;
            }
//        if (peer->bunch_ref.peer != inst.bunch_ref.peer) {
//        }
//        else {

//    inst.mark = travers_mark;
//    if (curr->inst_ref->mark == travers_mark) step /=2;

//if (curr->inst_ref.peer->makeName() == "$abc$712025$auto$blifparse.cc:535:parse_blif$718286") {
//    std::print("\n!!!!!!!!!!!!!!!!!! attracting from {} ({})", inst.makeName(), inst.cell_ref->type);
//}

            if (step > step_x/5 && curr->inst_ref.peer != exclude/* && curr->inst_ref->bunch_ref.peer != bunch*/) {
//if (curr->inst_ref.peer->makeName() == "$abc$712025$auto$blifparse.cc:535:parse_blif$718286") {
//    std::print("\n!!!!!!!!!!!!!!!!!!");
//}
                attractInst(*curr->inst_ref.peer, bunch, step/2, x, y, i, exclude, depth + 1);
            }
//        }
        }
    }
}

void OutlineDesign::recurseDrawDesign(rtl::Inst& inst, RegBunch* bunch, int mode, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

if (mode == 0) {
    if (inst.cell_ref->type.find("LUT") != std::string::npos) {
        image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 255, 0, 255);
    }
    else {
        image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 0, 255, 255);
    }
}

//if (inst.outline.x < 0.3 && inst.outline.y < 0.3) std::print("\n----------------- {} ({}) {} {}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y);


    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // excluding clock ports
                continue;
            }

            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
                continue;
            }

            rtl::Inst* peer = curr->inst_ref.peer;

            if (peer->outline.fixed || curr->inst_ref->outline.fixed) {
                image.draw_line(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, peer->outline.x*aspect_x*image_zoom, peer->outline.y*aspect_y*image_zoom, 200, 200, 200, 100);
            }
            else
            if (peer->bunch_ref.peer != inst.bunch_ref.peer) {
if (mode == 1) {
                image.draw_line(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, peer->outline.x*aspect_x*image_zoom, peer->outline.y*aspect_y*image_zoom, 255, 0, 0, 100);
}
            }
            else {
if (mode == 1) {
                image.draw_line(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, peer->outline.x*aspect_x*image_zoom, peer->outline.y*aspect_y*image_zoom, 0, 200, 200, 100);
}
            }

            if (peer->mark != travers_mark) {
//                peer->mark = travers_mark;
                recurseDrawDesign(*peer, nullptr, mode, depth + 1);
            }
        }
    }


    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseDrawDesign(*subbunch.reg, &subbunch, mode, depth + 1);
        }
    }
}

void OutlineDesign::recurseDumpDesign(rtl::Inst& inst, RegBunch* bunch, FILE* out, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

    std::print(out, "{} ({}), coords: {},{}\n", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y);

    if (inst.cell_ref->type.find("LUT") != std::string::npos) {
        image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 255, 0, 255);
    }
    else {
        image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 0, 255, 255);
    }

    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // excluding clock ports
                continue;
            }

            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
                continue;
            }

            rtl::Inst* peer = curr->inst_ref.peer;

            if (peer->mark != travers_mark) {
//                peer->mark = travers_mark;
                recurseDumpDesign(*peer, nullptr, out, depth + 1);
            }
        }
    }


    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseDumpDesign(*subbunch.reg, &subbunch, out, depth + 1);
        }
    }
}

