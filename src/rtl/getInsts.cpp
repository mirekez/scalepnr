#include "GetInsts.h"
#include "Port.h"
#include "Cell.h"
#include "Module.h"
#include "debug.h"

using namespace rtl;

bool rtl::compare(const std::string& value, const std::string& mask, bool partial, bool regexp, re2::RE2& regex)
{
    if (partial && value.find(mask) != std::string::npos) {
        return true;
    }
    else
    if (!regexp && value == mask) {
        return true;
    }
    else {
        if (regexp && re2::RE2::FullMatch(value, regex)) {
            return true;
        }
    }
    return false;
}

void rtl::getInsts(std::vector<Inst*>* insts, std::vector<instFilter>& filters, Referable<Inst>* inst, int depth)
{
    if (depth == 0) {
        PNR_LOG1("RTGI", "getInsts, inst: '{}', cell_name: '{}', type: '{}', is_blackbox: {}",
            inst->makeName(), inst->cell_ref->name, inst->cell_ref->type, inst->cell_ref->module_ref->is_blackbox);
    }
    else {
        PNR_LOG2("RTGI", "inst: '{}', cell_name: '{}', type: '{}', is_blackbox: {}",
            inst->makeName(), inst->cell_ref->name, inst->cell_ref->type, inst->cell_ref->module_ref->is_blackbox);
    }

    bool found = false;
    int i = -1;
    for (auto& filter : filters) {
        ++i;
        if (depth == 0) {
            PNR_LOG2("RTGI", "filter: {}", filter.format());
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

        bool filter_allowed = true;
        std::string inst_name = inst->makeName();
        if (filter.name.length() && !compare(inst_name, filter.name, filter.partial, filter.regexp, *filter.name_regex.get())) {
            filter_allowed = false;
        }
        if (filter.cell_name.length() && !compare(inst->cell_ref->name, filter.cell_name, filter.partial, filter.regexp, *filter.cell_regex.get())) {
            filter_allowed = false;
        }
        if (filter.cell_type.length() && !compare(inst->cell_ref->type, filter.cell_type, filter.partial, filter.regexp, *filter.type_regex.get())) {
            filter_allowed = false;
        }
        if (filter_allowed && filter.port_name.length())
        {
            bool found_port = false;
            for (auto& conn : inst->conns) {
                std::string port_name = conn.makeName(&inst_name);
                if (i == 0) {
                    PNR_LOG3("RTGI", " '{}'('{}')", port_name, conn.port_ref->name);
                }
                if (compare(port_name, filter.port_name, filter.partial, filter.regexp, *filter.port_regex.get())) {
                    found_port = true;
                    PNR_LOG1("RTGI", "FOUND PORT! '{}' ('{}')", port_name, conn.port_ref->name);
                }
            }
            if (!found_port) {
                filter_allowed = false;
            }
        }
        if (filter_allowed) {
            found = true;
            PNR_LOG1("RTGI", "FOUND! inst: '{}', cell_name: '{}', type: '{}', is_blackbox: {}",
                inst->makeName(), inst->cell_ref->name, inst->cell_ref->type, inst->cell_ref->module_ref->is_blackbox);
        }
    }
    if (found) {
        insts->push_back(inst);
    }

    for (auto& sub_inst : inst->insts) {
        getInsts(insts, filters, &sub_inst, depth+1);
    }
}
