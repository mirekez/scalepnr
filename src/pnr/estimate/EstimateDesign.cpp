#include "EstimateDesign.h"
#include "on_return.h"

using namespace pnr;

void EstimateDesign::findTopOutputs(rtl::Design& rtl)
{
    PNR_LOG1("ESTM", "findTopOutputs(), buffers_ports: {}", tech->buffers_ports);
    auto& top = rtl.top;
    for (auto& conn : top.conns) {
        PNR_LOG2("ESTM", "conn '{}' ('{}')", conn.makeName(), conn.inst_ref->cell_ref->type);
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
                PNR_LOG2("ESTM", "adding '{}' ('{}')", curr->makeName(), curr->inst_ref->cell_ref->type);
                data_outs.push_back(RegBunch{curr->inst_ref.peer});
            }
        }
    }
}

void EstimateDesign::estimateDesign(rtl::Design& rtl)
{
    findTopOutputs(rtl);
    travers_mark = rtl::Inst::genMark();
    for (auto& data_out : data_outs) {
        recurseReg(&data_out, data_out.reg);
        aggregateRegs(&data_out);
    }

    sortBunches(&data_outs);
}

void EstimateDesign::printBunches(std::list<Referable<RegBunch>>* bunch_list, int depth)
{
    if (!bunch_list) {
        bunch_list = &data_outs;
    }
    for (auto& bunch : *bunch_list) {
        std::print("\n");
        for (int i=0; i < depth; ++i) std::print("  ");
        std::print("bunch '{}' ({}), size: {}, size_regs: {}/{}, size_comb: {}/{}, top_max_length: {}, top_max_comb: {}, top_max_delay: {:.3f}, max_deficit: {:.3f}, sub_bunches: {}, uplinks: {}",
            bunch.reg->makeName(), bunch.reg->cell_ref->type, bunch.size, bunch.size_regs_own, bunch.size_regs, bunch.size_comb_own, bunch.size_comb,
            bunch.reg->stats.top_max_length, bunch.reg->stats.top_max_comb, bunch.reg->stats.top_max_delay, bunch.reg->stats.max_deficit, bunch.sub_bunches.size(), bunch.uplinks.size());

        printBunches(&bunch.sub_bunches, depth + 1);
    }
}

struct CompareBunches
{
    bool operator()(Referable<RegBunch>& a, Referable<RegBunch>& b)
    {
        return a.reg->stats.max_deficit > b.reg->stats.max_deficit || (a.reg->stats.max_deficit == b.reg->stats.max_deficit && a.reg->stats.top_max_delay > b.reg->stats.top_max_delay);
    }
};

struct CompareLinks
{
    bool operator()(BunchLink& a, BunchLink& b)
    {
        return a.deficit > b.deficit || (a.deficit == b.deficit && a.delay > b.delay);
    }
};

void EstimateDesign::sortBunches(std::list<Referable<RegBunch>>* bunch_list, int depth)
{
    bunch_list->sort(CompareBunches());
    for (auto& bunch : *bunch_list) {
        bunch.uplinks.sort(CompareLinks());
        sortBunches(&bunch.sub_bunches, depth + 1);
    }
}


int EstimateDesign::aggregateRegs(Referable<RegBunch>* bunch, int depth, int count_empty)
{
    if (bunch->size_comb_own == 0 && bunch->sub_bunches.size() == 1) {
        ++count_empty;
    }
    else {
        count_empty = 0;
    }
    int aggr_count = 0;
    for (auto& subbunch : bunch->sub_bunches) {
        int ret = aggregateRegs(&subbunch, depth + 1, count_empty);
        bunch->size += subbunch.size;
        if (ret > 0) {
            aggr_count = aggr_count==0?ret:(aggr_count<ret?aggr_count:ret);
        }
        if (aggr_count > 1 && bunch->clk_ref.peer == subbunch.clk_ref.peer
            && subbunch.size_comb_own == 0 && subbunch.sub_bunches.size() == 1
            && bunch->size_comb_own == 0 && bunch->sub_bunches.size() == 1
            && bunch->uplinks.size() == 1 ) {
            subbunch.reg->bunch_ref.set(bunch);
            bunch->size_comb_own += subbunch.size_comb_own;  // must be 0
            bunch->size_regs_own += subbunch.size_regs_own;
            bunch->size = subbunch.size_regs;  // minus one bunch
            auto tmp1 = std::move(subbunch.sub_bunches);
            auto tmp2 = std::move(subbunch.uplinks);
            subbunch.sub_bunches.clear();
            subbunch.uplinks.clear();
            for (auto* ref : subbunch.getPeers()) {  // all insts who are referring to this subbunch we want to delete
                Ref<pnr::RegBunch>::fromBase(ref)->set(bunch);  // fix smart pointer, it should point to parent bunch
            }
            PNR_ASSERT(subbunch.getPeers().size() == 0, "EstimateDesign::aggregateRegs, internal error in smart pointers")
            bunch->sub_bunches.clear();  // delete subbanch !!!
            bunch->uplinks.clear();
            bunch->sub_bunches = std::move(tmp1);
            bunch->uplinks = std::move(tmp2);
            bunch->size -= 1;
            for (auto& subbunch: bunch->sub_bunches) {
                subbunch.parent = bunch;
            }
            PNR_LOG2_("ESTM", depth, "adding regs chain {} ({}), size: {}, size_comb: {}, size_comb_own: {}, subbunches: {}", bunch->reg->makeName(), bunch->reg->cell_ref->type,
                bunch->size_regs, bunch->size_comb, bunch->size_comb_own, bunch->sub_bunches.size());
            break;
        }
    }
    bunch->size += 1;
    bunch->size_regs_own += 1;
    if (count_empty > 4) {
        return 4;
    }
    PNR_LOG2_("ESTM", depth, "aggregateRegs, reg: {} ({}), count_empty: {}, ret: {}, size: {}, size_comb: {}, size_comb_own: {}, subbunches: {}", bunch->reg->makeName(), bunch->reg->cell_ref->type,
        count_empty, aggr_count>0?aggr_count-1:0, bunch->size_regs, bunch->size_comb, bunch->size_comb_own, bunch->sub_bunches.size());
    return aggr_count>0?aggr_count-1:0;
}

void EstimateDesign::recurseComb(Referable<RegBunch>* bunch, rtl::Inst* comb, rtl::Conn* from /*need to calculate delay*/, int depth, int depth_comb, double bottom_delay, bool capture)
{
    if (comb->bunch_ref.peer == nullptr) {
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
    if (comb->bunch_ref.peer != bunch) {  // capture it?
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
    PNR_LOG2_("ESTM", depth, "recursing comb '{}'('{}'), depth {}/{}, bottom_max_length: {}, bottom_max_comb: {}, bottom_max_delay: {:.3f}, capture: {}",
        comb->makeName(), comb->cell_ref->type, depth, depth_comb, comb->stats.bottom_max_length, comb->stats.bottom_max_comb, comb->stats.bottom_max_delay, capture);

    if (comb->bunch_ref.peer != nullptr/*comb->mark == travers_mark*/ && !capture) {  // already taken
        return;
    }
//    comb->mark = travers_mark;

    comb->bunch_ref.set(bunch);

    int top_max_length = 0;
    int top_max_comb = 0;
    double top_max_delay = 0;
    double max_deficit = -100;
    for (auto& conn : comb->conns) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            PNR_LOG3("ESTM", " '{}'/'{}'", curr->makeNetName(), curr->makeName());
            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
//                PNR_WARNING("cant trace conn '{}' of '{}' ('{}')", curr->makeName(), curr->inst_ref->cell_ref->name, curr->inst_ref->cell_ref->type);
                continue;
            }

            int index_in = from->port_ref->index;
            int index_out = curr->port_ref->index;
            double delay = tech->comb_delays.getDelay(curr->inst_ref->cell_ref->type, index_in, index_out);
            PNR_LOG2_("ESTM", depth, "got '{}'('{}')", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type);
            auto it1 = tech->clocked_ports.find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
            auto it2 = tech->buffers_ports.find(curr->inst_ref->cell_ref->type);
            if (it1 != tech->clocked_ports.end() || it2 != tech->buffers_ports.end()) {
                bunch->uplinks.push_back(BunchLink{.conn = curr, .delay = bottom_delay + delay, .deficit = (bunch->clk_ref.peer ? bottom_delay + delay - bunch->clk_ref->period_ns : 0), .length = depth_comb + 1});
                auto& subbunch = bunch->sub_bunches.emplace_back(RegBunch{curr->inst_ref.peer});
                if (!recurseReg(&subbunch, subbunch.reg, depth + 1, depth_comb + 1)) {
                    bunch->uplinks.back().secondary = true;
                    bunch->sub_bunches.pop_back();
                    continue;
                }
                subbunch.parent = bunch;
                bunch->size_comb += subbunch.size_comb;  // inherit all subbunches
                bunch->size_regs += subbunch.size_regs;  // inherit all subbunches
                // regs terminate top_max_comb info
                // regs terminate top_max_delays info
                if (curr->inst_ref->stats.top_max_length > top_max_length) {
                    top_max_length = curr->inst_ref->stats.top_max_length;
                }
                if (curr->inst_ref->stats.max_deficit > max_deficit) {
                    max_deficit = curr->inst_ref->stats.max_deficit;
                }
            }
            else {
//                bunch->members.push_back(curr->inst_ref.peer);
                recurseComb(bunch, curr->inst_ref.peer, curr, depth + 1, depth_comb + 1, bottom_delay + delay, capture);

                if (curr->inst_ref->stats.top_max_length > top_max_length) {
                    top_max_length = curr->inst_ref->stats.top_max_length;
                }
                if (curr->inst_ref->stats.top_max_comb > top_max_comb) {
                    top_max_comb = curr->inst_ref->stats.top_max_comb;
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
    bunch->size_comb_own += 1;
    comb->stats.top_max_length = top_max_length + 1;
    comb->stats.top_max_comb = top_max_comb + 1;
    comb->stats.top_max_delay = top_max_delay;
    comb->stats.max_deficit = max_deficit;
    PNR_LOG2_("ESTM", depth, "done comb '{}'('{}'), size_regs: {}, size_comb: {}, top_max_length: {}, top_max_comb: {}, top_max_delay: {:.3f}, max_deficit: {:.3f}, ",
        comb->makeName(), comb->cell_ref->type, bunch->size_regs, bunch->size_comb, comb->stats.top_max_length, comb->stats.top_max_comb, comb->stats.top_max_delay, comb->stats.max_deficit);
    return;
}

bool EstimateDesign::recurseReg(Referable<RegBunch>* bunch, rtl::Inst* reg, int depth, int depth_comb)
{
    if (reg->stats.bottom_max_length < depth) {
        reg->stats.bottom_max_length = depth;
    }

    if (reg->bunch_ref.peer) {  // this register already added to some bunch
        PNR_LOG2_("ESTM", depth, "already done reg '{}'('{}'), size_regs: {}, size_comb: {}, top_max_length: {}, top_max_delay: {:.3f}, max_deficit: {:.3f}",
            reg->makeName(), reg->cell_ref->type, bunch->size_regs, bunch->size_comb, reg->stats.top_max_length, reg->stats.top_max_delay, reg->stats.max_deficit);
        return false;
    }

    PNR_LOG2_("ESTM", depth, "recusing reg '{}'('{}'), depth {}/{}, bottom_max_length: {}", reg->makeName(), reg->cell_ref->type, depth, depth_comb, reg->stats.bottom_max_length);

    auto it1 = tech->clocked_ports.find(reg->cell_ref->type);
    if (it1 == tech->clocked_ports.end()) {
        auto it2 = tech->buffers_ports.find(reg->cell_ref->type);  // we count IOBUF here as clocked
        if (it2 == tech->buffers_ports.end()) {
            PNR_WARNING("internal error: cant find '{}'('{}') in clocked_ports {}", reg->makeName(), reg->cell_ref->type, tech->clocked_ports);
            return false;
        }
    }
//    bunch->size = 0;

//    if (reg->mark == travers_mark) {  // already taken
//        return;
//    }
//    reg->mark = travers_mark;

    reg->bunch_ref.set(bunch);

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
                                PNR_LOG2_("ESTM", depth, "found clock {} for '{}': '{}'", (uint64_t)&clk, clk_conn->makeName(), clk.name);
                                bunch->clk_ref.set(&clk);
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

            PNR_LOG3("ESTM", " '{}'/'{}'", curr->makeNetName(), curr->makeName());
            curr = curr->follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
//                PNR_WARNING("cant trace conn '{}' of '{}' ('{}')", curr->makeName(), curr->inst_ref->cell_ref->name, curr->inst_ref->cell_ref->type);
                continue;
            }
            PNR_LOG2_("ESTM", depth, "got '{}'('{}')", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type);
            auto it1 = tech->clocked_ports.find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
            auto it2 = tech->buffers_ports.find(curr->inst_ref->cell_ref->type);
            if (it1 != tech->clocked_ports.end() || it2 != tech->buffers_ports.end()) {
                bunch->uplinks.push_back(BunchLink{.conn = curr, .delay = 0, .deficit = (bunch->clk_ref.peer ? 0 - bunch->clk_ref->period_ns : 0), .length = 0});
                auto& subbunch = bunch->sub_bunches.emplace_back(RegBunch{curr->inst_ref.peer});
                if (!recurseReg(&subbunch, subbunch.reg, depth + 1, 0)) {
                    bunch->uplinks.back().secondary = true;
                    bunch->sub_bunches.pop_back();
                    continue;
                }
                subbunch.parent = bunch;
                bunch->size_comb += subbunch.size_comb;
                bunch->size_regs += subbunch.size_regs;
                if (curr->inst_ref->stats.top_max_length > top_max_length) {
                    top_max_length = curr->inst_ref->stats.top_max_length;
                }
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
    for (auto& subbunch : bunch->sub_bunches) {  // we need to take it here directly from subbanches to do not disturb delay calculation
        if (subbunch.reg->stats.top_max_comb > top_max_comb) {
            top_max_comb = subbunch.reg->stats.top_max_comb;
        }
        if (subbunch.reg->stats.top_max_delay > top_max_delay) {
            top_max_delay = subbunch.reg->stats.top_max_delay;
        }
    }
    reg->stats.top_max_length = top_max_length + 1;
    reg->stats.top_max_comb = top_max_comb;
    reg->stats.top_max_delay = top_max_delay;
    if (bunch->clk_ref.peer) {
        reg->stats.max_deficit = top_max_delay - bunch->clk_ref->period_ns;
        if (reg->stats.max_deficit < max_deficit) {
            reg->stats.max_deficit = max_deficit;
        }
    }
    else {
        reg->stats.max_deficit = max_deficit;
    }
    PNR_LOG2_("ESTM", depth, "done reg '{}'('{}'), size_regs: {}, size_comb: {}, top_max_length: {}, top_max_comb: {}, top_max_delay: {:.3f}, max_deficit: {:.3f}",
        reg->makeName(), reg->cell_ref->type, bunch->size_regs, bunch->size_comb, reg->stats.top_max_length, reg->stats.top_max_comb, reg->stats.top_max_delay, reg->stats.max_deficit);
    return true;
}
