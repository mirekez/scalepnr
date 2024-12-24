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

void Placing::recurseComb(Referable<rtl::RegBunch>* bunch, rtl::Inst* comb, rtl::CombStats* stats, bool clear, rtl::Conn* from, int depth_regs, int depth_comb)
{
    if (clear) {
        comb->used_in_bunches = 0;
    }
    else {
        ++comb->used_in_bunches;
    }

    PNR_LOG2_("PLCE", depth_comb, "recursing comb '{}'('{}'), clear: {}, used: {}, depth {}/{}", comb->makeName(), comb->cell_ref->type, clear, comb->used_in_bunches, depth_regs, depth_comb);
    int weight = 0;
    int size = 0;
    int max_length = 0;
    double max_delay = 0;
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

            if (curr->inst_ref->mark == mark) {  // already added
                continue;
            }
            curr->inst_ref->mark = mark;

            int index_in = from->port_ref->index;
            int index_out = curr->port_ref->index;
            double delay = tech->comb_delays.getDelay(curr->inst_ref->cell_ref->type, index_in, index_out);
            PNR_LOG2_("PLCE", depth_comb, "got '{}'('{}')", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type);
            auto it1 = tech->clocked_ports.find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
            auto it2 = tech->buffers_ports.find(curr->inst_ref->cell_ref->type);
            if (it1 != tech->clocked_ports.end() || it2 != tech->buffers_ports.end()) {
                bunch->sub_bunches.emplace_back(rtl::RegBunch{curr->inst_ref.peer});
                recurseReg(&bunch->sub_bunches.back(), clear, depth_regs + 1, depth_comb);
                size += bunch->sub_bunches.back().stats.size;
                weight += bunch->sub_bunches.back().stats.weight;
                if (bunch->sub_bunches.back().stats.max_length > max_length) {
                    max_length = bunch->sub_bunches.back().stats.max_length + 1;
                }
                // regs terminate max_delays info

                if (bunch->sub_bunches.back().stats.max_deficit > max_deficit) {
                    max_deficit = bunch->sub_bunches.back().stats.max_deficit;
                }
            }
            else {
                bunch->members.push_back(curr->inst_ref.peer);
                rtl::CombStats sub_stats;
                recurseComb(bunch, curr->inst_ref.peer, &sub_stats, clear, curr, depth_regs, depth_comb + 1);
                size += sub_stats.size;
                weight += sub_stats.weight;
                if (sub_stats.max_length + 1 > max_length) {
                    max_length = sub_stats.max_length + 1;
                }
                if (sub_stats.max_delay + delay > max_delay) {
                    max_delay = sub_stats.max_delay + delay;
                }
                if (sub_stats.max_deficit > max_deficit) {
                    max_deficit = sub_stats.max_deficit;
                }
            }
        }
    }
    stats->size = size + 1;
    stats->weight = weight + comb->used_in_bunches;
    stats->max_length = max_length;
    stats->max_delay = max_delay;
    stats->max_deficit = max_deficit;
    PNR_LOG2_("PLCE", depth_comb, "done comb '{}'('{}'), size: {}, weight: {}, max_length: {}, max_delay: {:.3f}, max_deficit: {:.3f}",
        comb->makeName(), comb->cell_ref->type, stats->size, stats->weight, stats->max_length, stats->max_delay, stats->max_deficit);
}

void Placing::recurseReg(Referable<rtl::RegBunch>* bunch, bool clear, int depth_regs, int depth_comb)
{
    if (depth_regs == 0 && depth_comb == 0) {
        mark = rtl::Inst::genMark();
    }
    if (bunch->reg_in->placing.peer) {
        bunch->stats = bunch->reg_in->placing->stats;
        PNR_LOG2_("PLCE", depth_comb, "already done reg '{}'('{}'), size: {}, weight: {}, max_length: {}, max_delay: {:.3f}, max_deficit: {:.3f}",
            bunch->reg_in->makeName(), bunch->reg_in->cell_ref->type, bunch->stats.size, bunch->stats.weight, bunch->stats.max_length, bunch->stats.max_delay, bunch->stats.max_deficit);
        return;
    }
    if (clear) {
    }
    else {
    }
    auto it1 = tech->clocked_ports.find(bunch->reg_in->cell_ref->type);
    if (it1 == tech->clocked_ports.end()) {
        auto it2 = tech->buffers_ports.find(bunch->reg_in->cell_ref->type);  // we count IOBUF here as clocked
        if (it2 == tech->buffers_ports.end()) {
            PNR_WARNING("internal error: cant find '{}'('{}') in clocked_ports {}", bunch->reg_in->makeName(), bunch->reg_in->cell_ref->type, tech->clocked_ports);
            return;
        }
    }
    PNR_LOG2_("PLCE", depth_comb, "recursing reg '{}'('{}'), clear: {}, depth {}/{}", bunch->reg_in->makeName(), bunch->reg_in->cell_ref->type, clear, depth_regs, depth_comb);
    int weight = 0;
    int size = 0;
    int max_length = 0;
    double max_delay = 0;
    double max_deficit = -100;
    for (auto& conn : bunch->reg_in->conns) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            auto clk_port_name = it1;
            while (clk_port_name != tech->clocked_ports.end()) {
                if (clk_port_name->second == curr->port_ref->name) {  // clock port // TODO: add support for 2-clock primitives
                    rtl::Conn* clk_conn = curr->follow();
                    if (clk_conn && clocks) {
                        for (auto& clk : clocks->clocks_list) {
                            if (clk.conn_ptr == clk_conn || clk.bufg_ptr == clk_conn->inst_ref.peer) {
                                PNR_LOG2_("PLCE", depth_comb, "found clock for '{}': '{}'", clk_conn->makeName(), clk.name);
                                bunch->clk_in = &clk;
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

            if (curr->inst_ref->mark == mark) {  // already added
                continue;
            }
            curr->inst_ref->mark = mark;

            PNR_LOG2_("PLCE", depth_comb, "got '{}'('{}')", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type);
            auto it1 = tech->clocked_ports.find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
            auto it2 = tech->buffers_ports.find(curr->inst_ref->cell_ref->type);
            if (it1 != tech->clocked_ports.end() || it2 != tech->buffers_ports.end()) {
                bunch->sub_bunches.emplace_back(rtl::RegBunch{curr->inst_ref.peer});
                recurseReg(&bunch->sub_bunches.back(), clear, depth_regs + 1, depth_comb);
                size += bunch->sub_bunches.back().stats.size;
                weight += bunch->sub_bunches.back().stats.weight;
                if (bunch->sub_bunches.back().stats.max_length > max_length) {
                    max_length = bunch->sub_bunches.back().stats.max_length + 1;
                }
                // regs terminate max_delays info

                if (bunch->sub_bunches.back().stats.max_deficit > max_deficit) {
                    max_deficit = bunch->sub_bunches.back().stats.max_deficit;
                }
            }
            else {
                bunch->members.push_back(curr->inst_ref.peer);
                rtl::CombStats sub_stats;
                recurseComb(bunch, curr->inst_ref.peer, &sub_stats, clear, curr, depth_regs, depth_comb + 1);
                size += sub_stats.size;
                weight += sub_stats.weight;
                if (sub_stats.max_length + 1 > max_length) {
                    max_length = sub_stats.max_length + 1;
                }
                if (sub_stats.max_delay > max_delay) {
                    max_delay = sub_stats.max_delay;
                }
                if (sub_stats.max_deficit > max_deficit) {
                    max_deficit = sub_stats.max_deficit;
                }
            }
        }
    }
    bunch->stats.size = size + 1;
    bunch->stats.weight = weight + 1;
    bunch->stats.max_length = max_length;
    bunch->stats.max_delay = max_delay;
    if (bunch->clk_in) {
        bunch->stats.max_deficit = max_delay - bunch->clk_in->period_ns;
        if (bunch->stats.max_deficit < max_deficit) {
            bunch->stats.max_deficit = max_deficit;
        }
    }
    else {
        bunch->stats.max_deficit = max_deficit;
    }
    bunch->reg_in->placing.set(bunch);
    PNR_LOG2_("PLCE", depth_comb, "done reg '{}'('{}'), size: {}, weight: {}, max_length: {}, max_delay: {:.3f}, max_deficit: {:.3f}",
        bunch->reg_in->makeName(), bunch->reg_in->cell_ref->type, bunch->stats.size, bunch->stats.weight, bunch->stats.max_length, bunch->stats.max_delay, bunch->stats.max_deficit);
}
