#include "Placing.h"
#include "on_return.h"

using namespace pnr;

void Placing::findTopOutputs(rtl::Design& rtl)
{
    PNR_LOG1("PLCE", "findTopOutputs(), buffers_ports: {}", tech->buffers_ports);
    auto& top = rtl.top;
    for (auto& conn : top.conns) {
        PNR_LOG2("PLCE", "conn '{}' ('{}')", conn.makeName(), conn.inst_ref->cell_ref->type);
        if (conn.port_ref->type == rtl::Port::PORT_OUT) {
            rtl::Conn* curr = conn.follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
//                PNR_WARNING("cant trace conn '{}' of '{}' ('{}')", curr->makeName(), curr->inst_ref->cell_ref->name, curr->inst_ref->cell_ref->type);
                continue;
            }
            auto it = tech->buffers_ports.find(curr->inst_ref->cell_ref->type);
            if (it != tech->buffers_ports.end()) {
                rtl::Conn* data_out = (*curr->inst_ref)[it->second];
                if (!data_out) {
                    PNR_WARNING("cant find port '{}' of inst '{}'('{}')", it->second, curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type);
                    continue;
                }
                PNR_LOG2("PLCE", "adding '{}' ('{}')", curr->makeName(), curr->inst_ref->cell_ref->type);
                data_outs.push_back(DataOut{curr->inst_ref.peer});
            }
        }
    }
}

void Placing::recurseComb(Referable<RegBunch>* bunch, rtl::Inst* comb, rtl::Conn* from /*need to calculate delay*/, int depth, int depth_comb, double bottom_delay, bool capture)
{
    if (comb->bunch.peer == nullptr) {
        if (comb->stats.bottom_max_length < depth) {
            comb->stats.bottom_max_length = depth;
        }
        if (comb->stats.bottom_max_comb < depth_comb) {
            comb->stats.bottom_max_comb = depth_comb;
        }
        if (comb->stats.bottom_max_delay < bottom_delay) {
            comb->stats.bottom_max_delay = bottom_delay;
        }
    }
    else
    if (comb->bunch.peer != bunch) {  // capture it?
        if (comb->stats.bottom_max_delay < bottom_delay) {  // our delay is higher, we want to capture all this logic
            comb->stats.bottom_max_delay = bottom_delay;
            capture = true;

            if (comb->stats.bottom_max_length < depth) {
                comb->stats.bottom_max_length = depth;
            }
            if (comb->stats.bottom_max_comb < depth_comb) {
                comb->stats.bottom_max_comb = depth_comb;
            }
        }
    }
    else {  // it's ours
//        capture = false;
    }
    PNR_LOG2_("PLCE", depth, "recursing comb '{}'('{}'), depth {}/{}, bottom_max_length: {}, bottom_max_comb: {}, bottom_max_delay: {:.3f}, capture: {}",
        comb->makeName(), comb->cell_ref->type, depth, depth_comb, comb->stats.bottom_max_length, comb->stats.bottom_max_comb, comb->stats.bottom_max_delay, capture);

    if (comb->bunch.peer != nullptr/*comb->mark == travers_mark*/ && !capture) {  // already taken
        return;
    }
//    comb->mark = travers_mark;

    comb->bunch.set(bunch);

    int top_max_length = 0;
    int top_max_comb = 0;
    double top_max_delay = 0;
    double max_deficit = -100;
    for (auto& conn : comb->conns) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            PNR_LOG3("PLCE", " '{}'/'{}'", curr->makeNetName(), curr->makeName());
            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
//                PNR_WARNING("cant trace conn '{}' of '{}' ('{}')", curr->makeName(), curr->inst_ref->cell_ref->name, curr->inst_ref->cell_ref->type);
                continue;
            }

            int index_in = from->port_ref->index;
            int index_out = curr->port_ref->index;
            double delay = tech->comb_delays.getDelay(curr->inst_ref->cell_ref->type, index_in, index_out);
            PNR_LOG2_("PLCE", depth, "got '{}'('{}')", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type);
            auto it1 = tech->clocked_ports.find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
            auto it2 = tech->buffers_ports.find(curr->inst_ref->cell_ref->type);
            if (it1 != tech->clocked_ports.end() || it2 != tech->buffers_ports.end()) {
                bunch->sub_bunches.emplace_back(RegBunch{curr->inst_ref.peer});
                recurseReg(&bunch->sub_bunches.back(), bunch->sub_bunches.back().reg, depth + 1, depth_comb + 1);

                bunch->size_comb += bunch->sub_bunches.back().size_comb;  // inherit all subbunches
                bunch->size_regs += bunch->sub_bunches.back().size_regs;  // inherit all subbunches
                if (curr->inst_ref->stats.top_max_length > top_max_length) {
                    top_max_length = curr->inst_ref->stats.top_max_length;
                }
                // regs terminate top_max_comb info
                // regs terminate top_max_delays info
                if (curr->inst_ref->stats.max_deficit > max_deficit) {
                    max_deficit = curr->inst_ref->stats.max_deficit;
                }
            }
            else {
//                bunch->members.push_back(curr->inst_ref.peer);
                recurseComb(bunch, curr->inst_ref.peer, curr, depth + 1, depth_comb + 1, bottom_delay + delay, capture);

                if (curr->inst_ref->stats.top_max_comb > top_max_comb) {
                    top_max_comb = curr->inst_ref->stats.top_max_comb;
                }
                if (curr->inst_ref->stats.top_max_length > top_max_length) {
                    top_max_length = curr->inst_ref->stats.top_max_length;
                }
                if (curr->inst_ref->stats.top_max_delay + delay > top_max_delay) {  // delay is different in each cycle
                    top_max_delay = curr->inst_ref->stats.top_max_delay + delay;
                }
                if (curr->inst_ref->stats.max_deficit > max_deficit) {
                    max_deficit = curr->inst_ref->stats.max_deficit;
                }
            }
        }
    }
    bunch->size_comb += 1;
    comb->stats.top_max_length = top_max_length + 1;
    comb->stats.top_max_comb = top_max_comb + 1;
    comb->stats.top_max_delay = top_max_delay;
    comb->stats.max_deficit = max_deficit;
    PNR_LOG2_("PLCE", depth, "done comb '{}'('{}'), size: {}/{}, top_max_length: {}, top_max_comb: {}, top_max_delay: {:.3f}, max_deficit: {:.3f}, ",
        comb->makeName(), comb->cell_ref->type, bunch->size_comb, bunch->size_regs, comb->stats.top_max_length, comb->stats.top_max_comb, comb->stats.top_max_delay, comb->stats.max_deficit);
}

void Placing::recurseReg(Referable<RegBunch>* bunch, rtl::Inst* reg, int depth, int depth_comb)
{
    if (reg->stats.bottom_max_length < depth) {
        reg->stats.bottom_max_length = depth;
    }

    if (reg->bunch.peer) {  // this register already added to a bunch
        PNR_LOG2_("PLCE", depth, "already done reg '{}'('{}'), size: {}/{}, top_max_length: {}, top_max_delay: {:.3f}, max_deficit: {:.3f}",
            reg->makeName(), reg->cell_ref->type, bunch->size_comb, bunch->size_regs, reg->stats.top_max_length, reg->stats.top_max_delay, reg->stats.max_deficit);
        return;
    }

    PNR_LOG2_("PLCE", depth, "recusing reg '{}'('{}'), depth {}/{}, bottom_max_length: {}", reg->makeName(), reg->cell_ref->type, depth, depth_comb, reg->stats.bottom_max_length);

    auto it1 = tech->clocked_ports.find(reg->cell_ref->type);
    if (it1 == tech->clocked_ports.end()) {
        auto it2 = tech->buffers_ports.find(reg->cell_ref->type);  // we count IOBUF here as clocked
        if (it2 == tech->buffers_ports.end()) {
            PNR_WARNING("internal error: cant find '{}'('{}') in clocked_ports {}", reg->makeName(), reg->cell_ref->type, tech->clocked_ports);
            return;
        }
    }
    bunch->size_regs = 0;

//    if (reg->mark == travers_mark) {  // already taken
//        return;
//    }
//    reg->mark = travers_mark;

    reg->bunch.set(bunch);

    int top_max_length = 0;
    int top_max_comb = 0;
    double top_max_delay = 0;
    double max_deficit = -100;
    for (auto& conn : reg->conns) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            auto clk_port_name = it1;
            while (clk_port_name != tech->clocked_ports.end()) {  // look for CLK port
                if (clk_port_name->second == curr->port_ref->name) {  // clock port // TODO: add support for 2-clock primitives
                    rtl::Conn* clk_conn = curr->follow();
                    if (clk_conn && clocks) {
                        for (auto& clk : clocks->clocks_list) {  // TODO: precalculate (cache) this
                            if (clk.conn_ptr == clk_conn || clk.bufg_ptr == clk_conn->inst_ref.peer) {
                                PNR_LOG2_("PLCE", depth, "found clock {} for '{}': '{}'", (uint64_t)&clk, clk_conn->makeName(), clk.name);
                                bunch->clk.set(&clk);
                                reg->cnt_clocks = 1;  // need support for 2-clk prims
                                break;
                            }
                        }
                    }
                    break;
                }
                ++clk_port_name;
            }
            if (clk_port_name != tech->clocked_ports.end()) {  // excluding clock ports
                continue;
            }

            PNR_LOG3("PLCE", " '{}'/'{}'", curr->makeNetName(), curr->makeName());
            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
//                PNR_WARNING("cant trace conn '{}' of '{}' ('{}')", curr->makeName(), curr->inst_ref->cell_ref->name, curr->inst_ref->cell_ref->type);
                continue;
            }
            PNR_LOG2_("PLCE", depth, "got '{}'('{}')", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type);
            auto it1 = tech->clocked_ports.find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
            auto it2 = tech->buffers_ports.find(curr->inst_ref->cell_ref->type);
            if (it1 != tech->clocked_ports.end() || it2 != tech->buffers_ports.end()) {
                bunch->sub_bunches.emplace_back(RegBunch{curr->inst_ref.peer});
                recurseReg(&bunch->sub_bunches.back(), bunch->sub_bunches.back().reg, depth + 1, 0);
                bunch->size_comb += bunch->sub_bunches.back().size_comb;  // inherit all subbunches
                bunch->size_regs += bunch->sub_bunches.back().size_regs;  // inherit all subbunches
                if (curr->inst_ref->stats.top_max_length > top_max_length) {
                    top_max_length = curr->inst_ref->stats.top_max_length;
                }
                // regs terminate top_max_comb info
                // regs terminate top_max_delays info
                if (curr->inst_ref->stats.max_deficit > max_deficit) {
                    max_deficit = curr->inst_ref->stats.max_deficit;
                }
            }
            else {
//                bunch->members.push_back(curr->inst_ref.peer);
                recurseComb(bunch, curr->inst_ref.peer, curr, depth + 1, 0, 0.0);
                if (curr->inst_ref->stats.top_max_length > top_max_length) {
                    top_max_length = curr->inst_ref->stats.top_max_length;
                }
                if (curr->inst_ref->stats.top_max_comb > top_max_comb) {
                    top_max_comb = curr->inst_ref->stats.top_max_comb;
                }
                if (curr->inst_ref->stats.top_max_delay > top_max_delay) {
                    top_max_delay = curr->inst_ref->stats.top_max_delay;
                }
                if (curr->inst_ref->stats.max_deficit > max_deficit) {
                    max_deficit = curr->inst_ref->stats.max_deficit;
                }
            }
        }
    }
    bunch->size_regs += 1;
    reg->stats.top_max_length = top_max_length + 1;
    for (auto& subbunch : bunch->sub_bunches) {
        if (subbunch.reg->stats.top_max_comb > top_max_comb) {
            top_max_comb = subbunch.reg->stats.top_max_comb;
        }
        if (subbunch.reg->stats.top_max_delay > top_max_delay) {
            top_max_delay = subbunch.reg->stats.top_max_delay;
        }
    }
    reg->stats.top_max_comb = top_max_comb;
    reg->stats.top_max_delay = top_max_delay;
    if (bunch->clk.peer) {
        reg->stats.max_deficit = top_max_delay - bunch->clk->period_ns;
        if (reg->stats.max_deficit < max_deficit) {
            reg->stats.max_deficit = max_deficit;
        }
    }
    else {
        reg->stats.max_deficit = max_deficit;
    }
    PNR_LOG2_("PLCE", depth, "done reg '{}'('{}'), size: {}/{}, top_max_length: {}, top_max_comb: {}, top_max_delay: {:.3f}, max_deficit: {:.3f}",
        reg->makeName(), reg->cell_ref->type, bunch->size_comb, bunch->size_regs, reg->stats.top_max_length, reg->stats.top_max_comb, reg->stats.top_max_delay, reg->stats.max_deficit);
}
/*
bool packBunch(Inst* inst, TileSet& set, int x = 0, int y = 0, int depth = 0)
{
    if (depth == 0) {
        mark = rtl::Inst::genMark();
    }
    PNR_LOG2_("PLCE", depth, "packBunch '{}'('{}'), depth {}", inst->makeName(), inst->cell_ref->type, depth);

    bool placed = false;
    for (auto& tile : set.tiles) {
        if (tile.x != x && tile.y != y) {
            if (tile.tryPlace(inst)) {
                placed = true;
            }
        }
    }
    if (!placed) {
        int max_x = 0, max_y = 0;
        for (auto& tile : set.tiles) {
            if (tile.coord.x > max_x) {
                max_x = tile.coord.x;
            }
            if (tile.coord.y > max_y) {
                max_y = tile.coord.y;
            }
        }
        int x = 0, y = 0;
        bool found = false;
        for (auto& tile : set.tiles) {
            if (tile.coord.x == max_x && y == tile.coord.y) {
                ++y;
            }
            if (tile.coord.x == max_x && tile.coord.y > y ) {
                found = true;
                x = max_x;
                break;
            }
        }
        if (!found) {
            for (auto& tile : set.tiles) {
                if (tile.coord.y == max_y && y == tile.coord.y) {
                    ++y;
                }
                if (tile.coord.x == max_x && tile.coord.y > y ) {
                    found = true;
                    break;
                }
            }
        }

        set.emplace_back(Referable<Tile>{});
        if (!set.back().tryPlace(inst)) {
            PNR_ERROR("cant place inst {} to empty tile {}", inst->makeName(), set.back().makeName());
            return false;
        }
    }

    for (auto& conn : inst->conns) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            auto clk_port_name = it1;
            while (clk_port_name != tech->clocked_ports.end()) {
                if (clk_port_name->second == curr->port_ref->name) {  // clock port // TODO: add support for 2-clock primitives
                    rtl::Conn* clk_conn = curr->follow();
                    if (clk_conn && clocks) {
                        for (auto& clk : clocks->clocks_list) {
                            if (clk.conn_ptr == clk_conn || clk.bufg_ptr == clk_conn->inst_ref.peer) {
                                PNR_LOG2_("PLCE", depth, "found clock for '{}': '{}'", clk_conn->makeName(), clk.name);
//?                                bunch->clk_in = &clk;
                                inst->cnt_clocks = 1;  // need support for 2-clk prims
                                break;
                            }
                        }
                    }
                    break;
                }
                ++clk_port_name;
            }
            if (clk_port_name != tech->clocked_ports.end()) {  // excluding clock ports
                continue;
            }

            PNR_LOG3("PLCE", " '{}'/'{}'", curr->makeNetName(), curr->makeName());
            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
//                PNR_WARNING("cant trace conn '{}' of '{}' ('{}')", curr->makeName(), curr->inst_ref->cell_ref->name, curr->inst_ref->cell_ref->type);
                continue;
            }

            if (curr->inst_ref->mark == mark) {  // already packed
                continue;
            }
            curr->inst_ref->mark = mark;

            PNR_LOG2_("PLCE", depth, "got '{}'('{}')", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type);
            auto it1 = tech->clocked_ports.find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
            auto it2 = tech->buffers_ports.find(curr->inst_ref->cell_ref->type);
            if (it1 == tech->clocked_ports.end() && it2 == tech->buffers_ports.end()) {
                if (!packBunch(curr->inst_ref.peer, tiles, depth + 1)) {
                    return false;
                }
            }
        }
    }
}
*/