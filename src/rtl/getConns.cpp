#include "GetConns.h"
#include "Port.h"
#include "Cell.h"
#include "Module.h"
#include "debug.h"

using namespace rtl;

void rtl::getConns(std::vector<Referable<Conn>*>* conns, std::vector<connFilter>& filters, Referable<Inst>* inst, int depth)
{
    if (depth == 0) {
        PNR_LOG1("RTLC", "getConns, inst: '{}', cell_name: '{}', type: '{}', is_blackbox: {}",
            inst->makeName(), inst->cell_ref->name, inst->cell_ref->type, inst->cell_ref->module_ref->is_blackbox);
    }
    else {
        PNR_LOG2("RTLC", "inst: '{}', cell_name: '{}', type: '{}', is_blackbox: {}",
            inst->makeName(), inst->cell_ref->name, inst->cell_ref->type, inst->cell_ref->module_ref->is_blackbox);
    }

    std::vector<bool> filter_allowed;
    int i = -1;
    for (auto& filter : filters) {
        ++i;
        filter_allowed.resize(i+1);
        if (depth == 0) {
            PNR_LOG2("RTLC", "filter: {}", filter.format());
        }

        if (filter.blackbox && !inst->cell_ref->module_ref->is_blackbox) {
            continue;
        }

        if (filter.regexp && filter.name.length() && !filter.name_regex.get()) {
            filter.name_regex.reset(new re2::RE2(filter.name));
        }
        if (filter.regexp && filter.cell_name.length() && !filter.cell_regex.get()) {
            filter.cell_regex.reset(new re2::RE2(filter.cell_name));
        }
        if (filter.regexp && filter.cell_type.length() && !filter.type_regex.get()) {
            filter.type_regex.reset(new re2::RE2(filter.cell_type));
        }
        if (filter.regexp && filter.port_name.length() && !filter.port_regex.get()) {
            filter.port_regex.reset(new re2::RE2(filter.port_name));
        }

        bool found_filter = true;
        std::string inst_name = inst->makeName();
        if (filter.name.length() && !rtl::compare(inst_name, filter.name, filter.partial, filter.regexp, *filter.name_regex.get())) {
            found_filter = false;
        }
        if (filter.cell_name.length() && !rtl::compare(inst->cell_ref->name, filter.cell_name, filter.partial, filter.regexp, *filter.cell_regex.get())) {
            found_filter = false;
        }
        if (filter.cell_type.length() && !rtl::compare(inst->cell_ref->type, filter.cell_type, filter.partial, filter.regexp, *filter.type_regex.get())) {
            found_filter = false;
        }
        if (found_filter) {
            filter_allowed[i] = true;
        }

        if (i == (int)filters.size()-1) {
            PNR_LOG2("RTLC", "inst: '{}', cell_name: '{}', type: '{}', is_blackbox: {}, filter_allowed: {}",
                inst->makeName(), inst->cell_ref->name, inst->cell_ref->type, inst->cell_ref->module_ref->is_blackbox, filter_allowed);
        }
    }

    for (auto& conn : inst->conns) {
        std::string port_name = conn.makeName();
        int i = -1;
        PNR_LOG3("RTLC", " '{}'('{}')", port_name, conn.port_ref->name);
        bool found = false;
        for (auto& filter : filters) {
            ++i;
            if (filter_allowed[i] && (!filter.port_name.length() || compare(port_name, filter.port_name, filter.partial, filter.regexp, *filter.port_regex.get()))) {
                found = true;
            }
        }
        if (found) {
            PNR_LOG1("RTLC", "FOUND PORT! '{}' ('{}')", port_name, conn.port_ref->name);
            conns->push_back(&conn);
        }
    }

    for (auto& sub_inst : inst->insts) {
        rtl::getConns(conns, filters, &sub_inst, depth+1);
    }
}

void rtl::getConns(std::vector<Referable<Conn>*>* conns, connFilter&& filter, Referable<Inst>* inst)
{
    std::vector<connFilter> filters;
    filters.emplace_back(std::move(filter));
    getConns(conns, filters, inst);
}
