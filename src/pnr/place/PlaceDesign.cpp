#include "PlaceDesign.h"
#include "Device.h"
#include "Tech.h"
#include "on_return.h"

using namespace pnr;

namespace {

bool isCarry(rtl::Inst& inst)
{
    return inst.cell_ref.peer && inst.cell_ref->type.find("CARRY") != std::string::npos;
}

rtl::Inst* carryChainDriver(rtl::Inst& inst)
{
    if (!isCarry(inst)) {
        return nullptr;
    }

    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->name != "CI") {
            continue;
        }

        rtl::Conn* driver_conn = conn.follow();
        if (!driver_conn || !driver_conn->inst_ref.peer) {
            continue;
        }

        rtl::Inst& driver = *driver_conn->inst_ref;
        if (isCarry(driver)) {
            return &driver;
        }
    }

    return nullptr;
}

bool carryChainPreferredCoord(rtl::Inst& inst, Coord& coord)
{
    rtl::Inst* driver = carryChainDriver(inst);
    if (!driver || !driver->tile.peer) {
        return false;
    }

    coord = driver->coord + Coord{0, -1};
    return true;
}

}

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

    if (rtl::Inst* driver = carryChainDriver(inst); driver && driver->mark != travers_mark) {
        recursivePackBunch(*driver, nullptr, depth + 1);
    }

    if (inst.cell_ref->type.find("FD") != (size_t)-1
        || inst.cell_ref->type.find("LUT") != (size_t)-1
        || inst.cell_ref->type.find("CARRY") != (size_t)-1
        || inst.cell_ref->type.find("MUX") != (size_t)-1) {

        if (inst.tile.peer) {
            inst.coord = inst.tile->coord;
            inst.outline.x = inst.coord.x + 0.25f*(inst.pos%4);
            inst.outline.y = inst.coord.y + 0.25f*(inst.pos/4);
            PNR_LOG2_("PLCE", depth, "inst already packed: '{}' ({}) to {} {}, pos: {}", inst.makeName(), inst.cell_ref->type,
                inst.coord.x, inst.coord.y, inst.pos);
        }
        else {
            Coord coord = {x,y};
            if (carryChainPreferredCoord(inst, coord)) {
                PNR_LOG2_("PLCE", depth, "packBunch, carry chain preferred coord for '{}': {} {}", inst.makeName(), coord.x, coord.y);
            }
            int dir = 0, steps = 1, search_pos = 0, placed_pos = 0;
            int i;
            constexpr int max_place_search_radius = 500;
            for (i=0; i < max_place_search_radius; ++i) {
                if (coord.x < 0 || coord.x >= fpga_width ||
                    coord.y < 0 || coord.y >= fpga_height ||
                    (*tile_grid)[coord.y*fpga_width+coord.x].coord.x == -1 ||
                    (*tile_grid)[coord.y*fpga_width+coord.x].coord.y == -1) {

                    radialSearch(coord, dir, steps, search_pos);
                    continue;
                }
//std::print("\neeeeeeeeeeeeeee {}", inst.makeName());
                if ((placed_pos = (*tile_grid)[coord.y*fpga_width+coord.x].tryAdd(&inst)) >= 0) {
                    PNR_LOG2_("PLCE", depth, "put inst: '{}' ({}), x: {}, y: {} to {} {}, pos: {}", bunch ? bunch->reg->makeName() : "-", inst.makeName(), inst.cell_ref->type,
                        x, y, coord.x, coord.y, placed_pos);
                    inst.coord = coord;
                    inst.outline.x = (coord.x + 0.25*(placed_pos%4))/aspect_x;  // just for drawing
                    inst.outline.y = (coord.y + 0.25*(placed_pos/4))/aspect_y;
                    inst.pos = placed_pos;
                    break;
                }
                radialSearch(coord, dir, steps, search_pos);
            }

            if (i == max_place_search_radius) {
                PNR_LOG2_("PLCE", depth, "cant place inst: '{}' ({}), coord: {}:{} => {}:{} => {}:{}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y, x, y, coord.x, coord.y);
                std::print("cant place inst: '{}' ({}), coord: {}:{} => {}:{} => {}:{}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y, x, y, coord.x, coord.y);
                exit(1);
            }
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
    PNR_LOG1("PLCE", "placeDesign");
    fpga = &fpga::Device::current();
    tile_grid = &fpga->tile_grid;

    int total_bunches = 0;
    int total_regs = 0;
    int total_comb = 0;

    for (auto& bunch : bunch_list) {
        total_bunches += bunch.size;
        total_regs += bunch.size_regs;
        total_comb += bunch.size_comb;  // need size of CARRY, MUX, SRL?   // then think about BRAM, LRAM, DSP
    }
//    combs_per_box = /*total_comb*/(float)fpga->cnt_luts / (mesh_width*mesh_height);

    fpga_width = fpga->size_width;
    fpga_height = fpga->size_height;

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
    image.write("place_output.png");
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

//    std::print("set_property LOC SLICE_X{}Y{} [get_cells {}]\n", (int)(inst.outline.x*aspect_x*aspect_x/10), (int)(inst.outline.y*aspect_y*aspect_y/10), inst.makeName(1000));

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
