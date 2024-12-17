#include "Placing.h"

using namespace pnr;

void Placing::findTopOutputs(rtl::Design& rtl, const std::multimap<std::string,std::string>& obufs_ports)
{
    PNR_LOG1("PLCE", "findTopOutputs(), obufs_ports: {}", obufs_ports);
    auto& top = rtl.top;
    for (auto& conn : top.conns) {
        PNR_LOG2("PLCE", "conn '{}' ('{}')", conn.makeName(), conn.inst_ref->cell_ref->type);
        if (conn.port_ref->type == rtl::Port::PORT_OUT) {
            rtl::Conn* curr = conn.follow();
            if (!curr || !curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global) {  // after BUFs (can be something?)
//                PNR_WARNING("cant trace conn '{}' of '{}' ('{}')", curr->makeName(), curr->inst_ref->cell_ref->name, curr->inst_ref->cell_ref->type);
                continue;
            }
            auto it = obufs_ports.find(curr->inst_ref->cell_ref->type);
            if (it != obufs_ports.end()) {
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

void Placing::recurseComb(Bunch* bunch, rtl::Inst* comb, int depth_regs, int depth_comb)
{
    PNR_LOG2_("PLCE", depth_comb, "recursing comb '{}'('{}'), depth {}/{}", comb->makeName(), comb->cell_ref->type, depth_regs, depth_comb);
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

            PNR_LOG2_("PLCE", depth_comb, "got '{}'('{}')", curr->inst_ref->makeName(), curr->inst_ref->cell_ref->type);
            auto it1 = clocked_ports->find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
            auto it2 = iobufs_ports->find(curr->inst_ref->cell_ref->type);
            if (it1 != clocked_ports->end() || it2 != iobufs_ports->end()) {
//                path->min_length = 0;
//                path->max_length = 0;
                bunch->sub_bunches.emplace_back(Bunch{curr->inst_ref.peer});
                recurseReg(&bunch->sub_bunches.back(), depth_regs + 1, depth_comb);
                continue;
            }
//    int min_length = 1000000000;
//    int max_length = -1;
            bunch->members.push_back(curr->inst_ref.peer);
            recurseComb(bunch, curr->inst_ref.peer, depth_regs, depth_comb + 1);
//                if (path->sub_paths.back().min_length < min_length) {
//                    min_length = path->sub_paths.back().min_length;
//                }
//                if (path->sub_paths.back().max_length > max_length) {
//                    max_length = path->sub_paths.back().max_length;
//                }
        }
    }
//    path->min_length = min_length + 1;
//    path->max_length = max_length + 1;
}


void Placing::recurseReg(Bunch* bunch, int depth_regs, int depth_comb)
{
    auto it1 = clocked_ports->find(bunch->reg_in->cell_ref->type);
    if (it1 == clocked_ports->end()) {
        auto it2 = iobufs_ports->find(bunch->reg_in->cell_ref->type);  // we count IOBUF here as clocked
        if (it2 == iobufs_ports->end()) {
            PNR_WARNING("internal error: cant find '{}'('{}') in clocked_ports {}", bunch->reg_in->makeName(), bunch->reg_in->cell_ref->type, *clocked_ports);
            return;
        }
    }
    PNR_LOG2_("PLCE", depth_comb, "recursing reg '{}'('{}'), depth {}/{}", bunch->reg_in->makeName(), bunch->reg_in->cell_ref->type, depth_regs, depth_comb);
    for (auto& conn : bunch->reg_in->conns) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            auto clk_port_name = it1;
            while (clk_port_name != clocked_ports->end()) {
                if (clk_port_name->second == curr->port_ref->name) {
                    break;
                }
                ++clk_port_name;
            }
            if (clk_port_name != clocked_ports->end()) {  // it's a clock port
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
            auto it1 = clocked_ports->find(curr->inst_ref->cell_ref->type);  // we support now only 100% clocked or 100% combinational BELs
            auto it2 = iobufs_ports->find(curr->inst_ref->cell_ref->type);
            if (it1 != clocked_ports->end() || it2 != iobufs_ports->end()) {
//                path->min_length = 0;
//                path->max_length = 0;
                bunch->sub_bunches.emplace_back(Bunch{curr->inst_ref.peer});
                recurseReg(&bunch->sub_bunches.back(), depth_regs + 1, depth_comb);
                continue;
            }
//    int min_length = 1000000000;
//    int max_length = -1;
            bunch->members.push_back(curr->inst_ref.peer);
            recurseComb(bunch, curr->inst_ref.peer, depth_regs, depth_comb + 1);
//                if (path->sub_paths.back().min_length < min_length) {
//                    min_length = path->sub_paths.back().min_length;
//                }
//                if (path->sub_paths.back().max_length > max_length) {
//                    max_length = path->sub_paths.back().max_length;
//                }
        }
    }
//    path->min_length = min_length + 1;
//    path->max_length = max_length + 1;
}
