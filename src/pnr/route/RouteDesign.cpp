#include "RouteDesign.h"
#include "Device.h"
#include "Tech.h"
#include "Wire.h"

using namespace pnr;

bool RouteDesign::tryNext(Tile& from, Tile& to, int from_pos, int to_pos, std::vector<Wire>& wire, int depth)
{
    if (depth == 1000) {
        return false;
    }

    wire.resize(depth+1);
    wire[depth].from = from.coord;
    wire[depth].to = to.coord;

    if (from.coord == to.coord) {
        int joint = -1;
        PNR_ASSERT(from.cb_type, "cb_type is NULL in tile '{}' at ({},{}) type {}\n", from.makeName(), from.coord.x, from.coord.y, (int)from.type);
        if (!from.cb_type->canIn(from_pos, to_pos, joint)) {
            PNR_LOG3_("ROUT", depth, "tryNext, from: {}, to: {}, from_pos: {}, to_pos: {}, !canIn", from.coord, to.coord, from_pos, to_pos);
            return false;
        }
        if (!to.cb.tryIn(from_pos, to_pos)) {
            PNR_LOG3_("ROUT", depth, "tryNext, from: {}, to: {}, from_pos: {}, to_pos: {}, !tryIn", from.coord, to.coord, from_pos, to_pos);
            return false;
        }
        PNR_LOG3_("ROUT", depth, "tryNext, from: {}, to: {}, from_pos: {}, to_pos: {}, success", from.coord, to.coord, from_pos, to_pos);
        return true;
    }

    int pos = 0;
    while ((pos = from.cb.iterateOut(from_pos, from.coord, to.coord, pos)) >= 0)
    {
        if (depth == 0) {
            int joint;
            if (!from.cb_type->canOut(from_pos, pos, joint)) {
                PNR_LOG3_("ROUT", depth, "tryNext, from: {}, to: {}, from_pos: {}, to_pos: {}, !canOut", from.coord, to.coord, from_pos, to_pos);
                return false;
            }
        }
        else {
            int joint;
            if (!from.cb_type->canJump(from_pos, pos, joint)) {
                PNR_LOG3_("ROUT", depth, "tryNext, from: {}, to: {}, from_pos: {}, to_pos: {}, !canJump", from.coord, to.coord, from_pos, to_pos);
                return false;
            }
        }

        Coord next = from.cb.makeJump(from.coord, pos);
        Tile* from1 = fpga->getTile(next.x, next.y);

        if (tryNext(*from1, to, from_pos, to_pos, wire, depth+1)) {
            return true;
        }
    }
    return false;
}

bool RouteDesign::routeNet(rtl::Inst& from, rtl::Inst& to, std::vector<Wire>& wire)
{
//    PNR_ASSERT(!from.tile.peer, "RouteDesign::tryOut, inst '%s' tile is not assigned", from.makeName())
//    PNR_ASSERT(!to.tile.peer, "RouteDesign::tryOut, inst '%s' tile is not assigned", to.makeName())
    if (!from.tile.peer || !to.tile.peer) {  // IOBUFs
        return true;
    }
    return tryNext(*from.tile, *to.tile, 0, 0, wire);
}

void RouteDesign::recursiveRouteBunch(rtl::Inst& inst, RegBunch* bunch, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }

    inst.mark = travers_mark;

    int x = inst.coord.x*aspect_x;
    int y = inst.coord.y*aspect_y;

    PNR_LOG2_("ROUT", depth, "routeBunch, bunch: '{}' inst: '{}' ({}), x: {}, y: {} => {} {}", bunch ? bunch->reg->makeName() : "-", inst.makeName(), inst.cell_ref->type,
        inst.coord.x, inst.coord.y, x, y);

    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // clock ports
                //route clocks
                continue;
            }

            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
                continue;
            }

            rtl::Inst* peer = curr->inst_ref.peer;
            std::vector<Wire> wire;
            if (routeNet(inst, *peer, wire)) {
                inst.wire = std::move(wire);
            }

            if (peer->mark != travers_mark) {
                recursiveRouteBunch(*peer, nullptr, depth + 1);
            }
        }
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recursiveRouteBunch(*subbunch.reg, &subbunch, depth + 1);
        }
    }
}

void RouteDesign::routeDesign(std::list<Referable<RegBunch>>& bunch_list)
{
    int total_bunches = 0;
    int total_regs = 0;
    int total_comb = 0;

    for (auto& bunch : bunch_list) {
        total_bunches += bunch.size;
        total_regs += bunch.size_regs;
        total_comb += bunch.size_comb;  // need size of CARRY, MUX, SRL?   // then think about BRAM, LRAM, DSP
    }
//    combs_per_box = /*total_comb*/(float)fpga.cnt_luts / (mesh_width*mesh_height);

    fpga_width = fpga->size_width;
    fpga_height = fpga->size_height;

    aspect_x = (float)fpga_width/mesh_width;
    aspect_y = (float)fpga_height/mesh_height;

    travers_mark = rtl::Inst::genMark();
    for (auto& bunch : bunch_list) {
        PNR_ASSERT(bunch.reg, "zero reg in bunch with address {}", (uint64_t)&bunch);
        recursiveRouteBunch(*bunch.reg, &bunch);
    }

    travers_mark = rtl::Inst::genMark();
    image.init(mesh_width*aspect_x*image_zoom, mesh_height*aspect_y*image_zoom);
    image.clear();
    for (auto& bunch : bunch_list) {
        recurseDrawDesign(*bunch.reg, &bunch);
    }
    image.write(std::string("route_output.png"));
}

void RouteDesign::recurseDrawDesign(rtl::Inst& inst, RegBunch* bunch, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

    for (auto& wire : inst.wire) {
        int r = wire.to.x > wire.from.x ? wire.to.x - wire.from.x : wire.from.x - wire.to.x;
        int g = wire.to.y > wire.from.y ? wire.to.y - wire.from.y : wire.from.y - wire.to.y;
        std::print("\neee {} {} {} {}", wire.from.x, wire.from.y, wire.to.x, wire.to.y);
        image.draw_line(wire.from.x*image_zoom, wire.from.y*image_zoom, wire.to.x*image_zoom, wire.from.y*image_zoom, r*100, g*100, 0, 255);
        image.draw_line(wire.to.x*image_zoom, wire.from.y*image_zoom, wire.to.x*image_zoom, wire.to.y*image_zoom, r*100, g*100, 0, 255);
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

/*            if (peer->coord.fixed || curr->inst_ref->coord.fixed) {
                image.draw_line(inst.coord.x*aspect_x*image_zoom, inst.coord.y*aspect_y*image_zoom, peer->coord.x*aspect_x*image_zoom, peer->coord.y*aspect_y*image_zoom, 200, 200, 200, 100);
            }
            else
            if (peer->bunch_ref.peer != inst.bunch_ref.peer) {
if (mode == 1) {
                image.draw_line(inst.coord.x*aspect_x*image_zoom, inst.coord.y*aspect_y*image_zoom, peer->coord.x*aspect_x*image_zoom, peer->coord.y*aspect_y*image_zoom, 255, 0, 0, 100);
}
            }
            else {
if (mode == 1) {
                image.draw_line(inst.coord.x*aspect_x*image_zoom, inst.coord.y*aspect_y*image_zoom, peer->coord.x*aspect_x*image_zoom, peer->coord.y*aspect_y*image_zoom, 0, 200, 200, 100);
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
