#include "PlaceDesign.h"
#include "Device.h"
#include "on_return.h"

using namespace pnr;

void PlaceDesign::recursivePackBunch(rtl::Inst& inst, RegBunch* bunch, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }

    inst.mark = travers_mark;

    int x = inst.outline.x*aspect_x;
    int y = inst.outline.y*aspect_y;

    PNR_LOG2_("PLCE", depth, "packBunch, bunch: '{}' inst: '{}' ({}), x: {}, y: {} => {} {}", bunch ? bunch->reg->makeName() : "-", inst.makeName(), inst.cell_ref->type,
        inst.outline.x, inst.outline.y, x, y);

    if (inst.cell_ref->type == "FDRE" || inst.cell_ref->type == "LUT" || inst.cell_ref->type == "CARRY4" || inst.cell_ref->type == "MUXF7") {

        int search_x1 = x;
        int search_y1 = y;
        int search_x2 = x;
        int search_y2 = y;
        int curr_x = x;
        int curr_y = y;
        int dir = 0;
        int t = 0;
        bool cant_find = false;
        for (t=0; t < 100000; ++t) {  // need to be optimized
            cant_find = true;
            int pos;
            if ((pos = (*tile_grid)[curr_y*fpga_width+curr_x].tryAdd(&inst)) > 0) {
                PNR_LOG2_("PLCE", depth, "put inst: '{}' ({}), x: {}, y: {} to {} {}", bunch ? bunch->reg->makeName() : "-", inst.makeName(), inst.cell_ref->type,
                    x, y, curr_x, curr_y);
                inst.outline.x = curr_x + 0.25*(pos%4);
                inst.outline.y = curr_y + 0.25*(pos/4);
                cant_find = false;
                break;
            }

            if (dir == 0) {
                if (curr_x < search_x2) {
                    ++curr_x;
                    cant_find = false;
                }
                else {
                    if (search_x2 < fpga_width-1) {
                        cant_find = false;
                        ++search_x2;
                    }
                    dir = 1;
                }
            }
            else
            if (dir == 1) {
                if (curr_y < search_y2) {
                    ++curr_y;
                    cant_find = false;
                }
                else {
                    if (search_y2 < fpga_height-1) {
                        cant_find = false;
                        ++search_y2;
                    }
                    dir = 2;
                }
            }
            else
            if (dir == 2) {
                if (curr_x > search_x1) {
                    --curr_x;
                    cant_find = false;
                }
                else {
                    if (search_x1 > 0) {
                        cant_find = false;
                        --search_x1;
                    }
                    dir = 3;
                }
            }
            else
            if (dir == 3) {
                if (curr_y > search_y1) {
                    --curr_y;
                    cant_find = false;
                }
                else {
                    if (search_y1 > 0) {
                        cant_find = false;
                        --search_y1;
                    }
                    dir = 0;
                }
            }
            if (cant_find) {
                break;
            }
        }
        if (t == 100000 || cant_find) {
            PNR_LOG2_("PLCE", depth, "cant place inst: '{}' ({}), x: {}, y: {} => {} {}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y, x, y);
        }
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
                recursivePackBunch(*peer, nullptr, depth + 1);
            }
        }
    }


    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recursivePackBunch(*subbunch.reg, &subbunch, depth + 1);
        }
    }
}

void PlaceDesign::placeDesign(std::list<Referable<RegBunch>>& bunch_list)
{
    auto& fpga = fpga::Device::current();
    tile_grid = &fpga.tile_grid;

    int total_bunches = 0;
    int total_regs = 0;
    int total_comb = 0;

    for (auto& bunch : bunch_list) {
        total_bunches += bunch.size;
        total_regs += bunch.size_regs;
        total_comb += bunch.size_comb;  // need size of CARRY, MUX, SRL?   // then think about BRAM, LRAM, DSP
    }
//    combs_per_box = /*total_comb*/(float)fpga.cnt_luts / (mesh_width*mesh_height);

    fpga_width = fpga.size_width;
    fpga_height = fpga.size_height;

    aspect_x = (float)fpga_width/mesh_width;
    aspect_y = (float)fpga_height/mesh_height;

    travers_mark = rtl::Inst::genMark();
    for (auto& bunch : bunch_list) {
        recursivePackBunch(*bunch.reg, &bunch);
    }

    travers_mark = rtl::Inst::genMark();
    image.init(mesh_width*aspect_x*image_zoom, mesh_height*aspect_y*image_zoom);
    image.clear();
    for (auto& bunch : bunch_list) {
        recurseDrawDesign(*bunch.reg, &bunch);
    }
    image.write(std::string("place_output.png"));
}

void PlaceDesign::recurseDrawDesign(rtl::Inst& inst, RegBunch* bunch, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

    if (inst.cell_ref->type.find("BUF") != std::string::npos) {
        image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 255, 255, 255);
    }
    else if (inst.cell_ref->type.find("LUT") != std::string::npos) {
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

/*            if (peer->outline.fixed || curr->inst_ref->outline.fixed) {
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
*/
            if (peer->mark != travers_mark) {
//                peer->mark = travers_mark;
                recurseDrawDesign(*peer, nullptr, depth + 1);
            }
        }
    }


    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseDrawDesign(*subbunch.reg, &subbunch, depth + 1);
        }
    }
}
