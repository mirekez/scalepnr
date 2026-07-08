#include "PlaceDesign.h"
#include "Device.h"
#include "Tech.h"
#include "on_return.h"

#include <cstdlib>
#include <limits>
#include <vector>

using namespace pnr;

namespace {

constexpr size_t FULL_NAME_LIMIT = std::numeric_limits<size_t>::max();

bool placeChainTraceMatches(const rtl::Inst& inst)
{
    const char* filter = std::getenv("SCALEPNR_PLACE_CHAIN_TRACE");
    if (!filter || !*filter) {
        return false;
    }
    std::string name = const_cast<rtl::Inst&>(inst).makeName(FULL_NAME_LIMIT);
    return name.find(filter) != std::string::npos;
}

bool isCarry(rtl::Inst& inst)
{
    return inst.cell_ref.peer && inst.cell_ref->type.find("CARRY") != std::string::npos;
}

bool isLut(rtl::Inst& inst)
{
    return inst.cell_ref.peer && inst.cell_ref->type.find("LUT") == 0;
}

bool isMuxF7(rtl::Inst& inst)
{
    return inst.cell_ref.peer && inst.cell_ref->type.find("MUXF7") == 0;
}

bool isMuxF8(rtl::Inst& inst)
{
    return inst.cell_ref.peer && inst.cell_ref->type.find("MUXF8") == 0;
}

bool isPackableLogic(rtl::Inst& inst)
{
    if (!inst.cell_ref.peer) {
        return false;
    }

    const std::string& type = inst.cell_ref->type;
    return type.find("FD") != std::string::npos
        || type.find("LUT") != std::string::npos
        || type.find("CARRY") != std::string::npos
        || type.find("MUX") != std::string::npos;
}

void collectInsts(rtl::Inst& inst, std::vector<rtl::Inst*>& insts)
{
    insts.push_back(&inst);
    for (auto& sub_inst : inst.insts) {
        collectInsts(sub_inst, insts);
    }
}

bool strictLocalChain(rtl::Inst& left, rtl::Inst& right)
{
    return (isLut(left) && isMuxF7(right)) || (isMuxF7(left) && isMuxF8(right));
}

bool strictLocalChainInput(rtl::Inst& left, rtl::Inst& right, rtl::Port* sink_port)
{
    // Only mux data inputs are strict tile-local arcs; selector inputs route through fabric.
    if (!sink_port || !strictLocalChain(left, right)) {
        return false;
    }
    return sink_port->name == "I0" || sink_port->name == "I1";
}

bool strictChainPreferredCoord(rtl::Inst& inst, Coord& coord, float aspect_x, float aspect_y)
{
    // Connected LUT->MUXF7 and MUXF7->MUXF8 arcs are tile-local, so use an already placed peer as anchor.
    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        rtl::Conn* driver_conn = conn.follow();
        rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
        if (driver && driver->tile.peer && strictLocalChainInput(*driver, inst, conn.port_ref.peer)) {
            coord = driver->coord;
            return true;
        }
    }

    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_OUT) {
            continue;
        }
        for (auto* sink_ref : rtl::Conn::getSinks(conn)) {
            rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            rtl::Inst* sink = sink_conn ? sink_conn->inst_ref.peer : nullptr;
            if (sink && sink->tile.peer && strictLocalChainInput(inst, *sink, sink_conn ? sink_conn->port_ref.peer : nullptr)) {
                coord = sink->coord;
                return true;
            }
            if (!sink || !strictLocalChainInput(inst, *sink, sink_conn ? sink_conn->port_ref.peer : nullptr)) {
                continue;
            }
            for (auto& input : sink->conns) {
                if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) {
                    continue;
                }
                rtl::Conn* driver_conn = input.follow();
                rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
                if (driver && driver != &inst && driver->tile.peer
                    && strictLocalChainInput(*driver, *sink, input.port_ref.peer)) {
                    coord = driver->coord;
                    return true;
                }
            }
            coord = {static_cast<int>(sink->outline.x*aspect_x), static_cast<int>(sink->outline.y*aspect_y)};
            return true;
        }
    }
    return false;
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

void visitStrictLocalChainDrivers(rtl::Inst& inst, auto&& visit)
{
    // MUX chain input drivers must be packed before the mux that consumes their tile-local output.
    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        rtl::Conn* driver_conn = conn.follow();
        rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
        if (driver && strictLocalChainInput(*driver, inst, conn.port_ref.peer)) {
            visit(*driver);
        }
    }
}

void visitStrictLocalChainSinks(rtl::Inst& inst, auto&& visit)
{
    // Once a chain producer is placed, immediately pack tile-local mux consumers beside it.
    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_OUT) {
            continue;
        }
        for (auto* sink_ref : rtl::Conn::getSinks(conn)) {
            rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            rtl::Inst* sink = sink_conn ? sink_conn->inst_ref.peer : nullptr;
            if (sink && strictLocalChainInput(inst, *sink, sink_conn ? sink_conn->port_ref.peer : nullptr)) {
                visit(*sink);
            }
        }
    }
}

void visitStrictLocalChainSiblingDrivers(rtl::Inst& inst, auto&& visit)
{
    // A mux input producer must bring the other strict mux producers before it commits.
    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_OUT) {
            continue;
        }
        for (auto* sink_ref : rtl::Conn::getSinks(conn)) {
            rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            rtl::Inst* sink = sink_conn ? sink_conn->inst_ref.peer : nullptr;
            if (!sink || !strictLocalChainInput(inst, *sink, sink_conn ? sink_conn->port_ref.peer : nullptr) || sink->tile.peer) {
                continue;
            }
            for (auto& input : sink->conns) {
                if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) {
                    continue;
                }
                rtl::Conn* driver_conn = input.follow();
                rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
                if (driver && driver != &inst && strictLocalChainInput(*driver, *sink, input.port_ref.peer)) {
                    if (isMuxF7(inst) && isMuxF7(*driver)) {
                        continue;
                    }
                    visit(*driver);
                }
            }
        }
    }
}

bool strictLocalChainDriversReady(rtl::Inst& inst)
{
    // Strict mux sinks are packable only after every tile-local producer has a tile.
    bool ready = true;
    visitStrictLocalChainDrivers(inst, [&](rtl::Inst& driver) {
        if (!driver.tile.peer) {
            ready = false;
        }
    });
    return ready;
}

bool hasLockedUnplacedStrictSiblingDriver(rtl::Inst& inst)
{
    // A locked unplaced sibling is the caller waiting for this chain lane to be reserved.
    bool found = false;
    visitStrictLocalChainSiblingDrivers(inst, [&](rtl::Inst& driver) {
        if (driver.locked && !driver.tile.peer) {
            found = true;
        }
    });
    return found;
}

bool hasLockedUnplacedStrictSink(rtl::Inst& inst)
{
    // A locked unplaced sink is the caller waiting for this producer to return.
    bool found = false;
    visitStrictLocalChainSinks(inst, [&](rtl::Inst& sink) {
        if (sink.locked && !sink.tile.peer) {
            found = true;
        }
    });
    return found;
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
    if (inst.locked) {
        return;
    }

    inst.locked = true;
    on_return clear_lock([&]() { inst.locked = false; });
    inst.mark = travers_mark;

    int x = inst.outline.x*aspect_x;
    int y = inst.outline.y*aspect_y;

    PNR_LOG2_("PLCE", depth, "packBunch, bunch: '{}' inst: '{}' ({}), x: {}, y: {} => {} {}", bunch ? bunch->reg->makeName() : "-", inst.makeName(), inst.cell_ref->type,
        inst.outline.x, inst.outline.y, x, y);
    bool trace_chain = placeChainTraceMatches(inst);
    if (trace_chain) {
        std::print("\nPLACE_CHAIN begin inst='{}' type='{}' placed={} coord=({}, {}) pos={} outline=({}, {})",
            inst.makeName(FULL_NAME_LIMIT), inst.cell_ref.peer ? inst.cell_ref->type : std::string{},
            inst.tile.peer != nullptr,
            inst.tile.peer ? inst.tile->coord.x : -1,
            inst.tile.peer ? inst.tile->coord.y : -1,
            inst.pos, inst.outline.x, inst.outline.y);
    }
    const bool return_to_locked_chain_peer =
        hasLockedUnplacedStrictSiblingDriver(inst) || hasLockedUnplacedStrictSink(inst);

    if (rtl::Inst* driver = carryChainDriver(inst); driver && (driver->mark != travers_mark || !driver->tile.peer)) {
        recursivePackBunch(*driver, nullptr, depth + 1);
    }
    visitStrictLocalChainDrivers(inst, [&](rtl::Inst& driver) {
        if (trace_chain || placeChainTraceMatches(driver)) {
            std::print("\nPLACE_CHAIN driver-before sink='{}' driver='{}' type='{}' placed={} coord=({}, {}) pos={}",
                inst.makeName(FULL_NAME_LIMIT), driver.makeName(FULL_NAME_LIMIT),
                driver.cell_ref.peer ? driver.cell_ref->type : std::string{},
                driver.tile.peer != nullptr,
                driver.tile.peer ? driver.tile->coord.x : -1,
                driver.tile.peer ? driver.tile->coord.y : -1,
                driver.pos);
        }
        if (driver.mark != travers_mark || !driver.tile.peer) {
            recursivePackBunch(driver, nullptr, depth + 1);
        }
        if (trace_chain || placeChainTraceMatches(driver)) {
            std::print("\nPLACE_CHAIN driver-after sink='{}' driver='{}' placed={} coord=({}, {}) pos={}",
                inst.makeName(FULL_NAME_LIMIT), driver.makeName(FULL_NAME_LIMIT),
                driver.tile.peer != nullptr,
                driver.tile.peer ? driver.tile->coord.x : -1,
                driver.tile.peer ? driver.tile->coord.y : -1,
                driver.pos);
        }
    });
    visitStrictLocalChainSiblingDrivers(inst, [&](rtl::Inst& driver) {
        if (trace_chain || placeChainTraceMatches(driver)) {
            std::print("\nPLACE_CHAIN sibling-before inst='{}' sibling='{}' type='{}' placed={} coord=({}, {}) pos={}",
                inst.makeName(FULL_NAME_LIMIT), driver.makeName(FULL_NAME_LIMIT),
                driver.cell_ref.peer ? driver.cell_ref->type : std::string{},
                driver.tile.peer != nullptr,
                driver.tile.peer ? driver.tile->coord.x : -1,
                driver.tile.peer ? driver.tile->coord.y : -1,
                driver.pos);
        }
        if (driver.mark != travers_mark || !driver.tile.peer) {
            recursivePackBunch(driver, nullptr, depth + 1);
        }
        if (trace_chain || placeChainTraceMatches(driver)) {
            std::print("\nPLACE_CHAIN sibling-after inst='{}' sibling='{}' placed={} coord=({}, {}) pos={}",
                inst.makeName(FULL_NAME_LIMIT), driver.makeName(FULL_NAME_LIMIT),
                driver.tile.peer != nullptr,
                driver.tile.peer ? driver.tile->coord.x : -1,
                driver.tile.peer ? driver.tile->coord.y : -1,
                driver.pos);
        }
    });
    if (!strictLocalChainDriversReady(inst)) {
        PNR_LOG2_("PLCE", depth, "defer strict local chain sink: '{}' ({})", inst.makeName(), inst.cell_ref->type);
        return;
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
            else if (strictChainPreferredCoord(inst, coord, aspect_x, aspect_y)) {
                PNR_LOG2_("PLCE", depth, "packBunch, strict local chain preferred coord for '{}': {} {}", inst.makeName(), coord.x, coord.y);
                if (trace_chain) {
                    std::print("\nPLACE_CHAIN preferred inst='{}' coord=({}, {})",
                        inst.makeName(FULL_NAME_LIMIT), coord.x, coord.y);
                }
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

    visitStrictLocalChainSiblingDrivers(inst, [&](rtl::Inst& driver) {
        if (driver.mark != travers_mark || !driver.tile.peer) {
            recursivePackBunch(driver, nullptr, depth + 1);
        }
    });
    if (return_to_locked_chain_peer) {
        return;
    }

    visitStrictLocalChainSinks(inst, [&](rtl::Inst& sink) {
        if (sink.mark != travers_mark || !sink.tile.peer) {
            recursivePackBunch(sink, nullptr, depth + 1);
        }
    });

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

            if (peer->mark != travers_mark || !peer->tile.peer) {
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

    std::vector<rtl::Inst*> all_insts;
    collectInsts(tech->design.top, all_insts);

    constexpr int max_cleanup_passes = 64;
    for (int pass = 0; pass < max_cleanup_passes; ++pass) {
        int before = 0;
        for (rtl::Inst* inst : all_insts) {
            if (isPackableLogic(*inst) && !inst->tile.peer) {
                ++before;
            }
        }
        if (before == 0) {
            break;
        }

        travers_mark = rtl::Inst::genMark();
        int placed = 0;
        for (rtl::Inst* inst : all_insts) {
            if (!isPackableLogic(*inst) || inst->tile.peer) {
                continue;
            }

            recursivePackBunch(*inst, nullptr);
            if (inst->tile.peer) {
                ++placed;
            }
        }

        int after = 0;
        for (rtl::Inst* inst : all_insts) {
            if (isPackableLogic(*inst) && !inst->tile.peer) {
                ++after;
            }
        }

        std::print("\nPLACE_SWEEP pass={} before={} placed={} after={}", pass + 1, before, placed, after);
        if (after == 0) {
            break;
        }
        if (placed == 0) {
            std::print("\nPLACE_SWEEP blocked first_unplaced:");
            int printed = 0;
            for (rtl::Inst* inst : all_insts) {
                if (isPackableLogic(*inst) && !inst->tile.peer) {
                    std::print("\n  {} ({})", inst->makeName(FULL_NAME_LIMIT), inst->cell_ref->type);
                    if (++printed >= 12) {
                        break;
                    }
                }
            }
            break;
        }
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
