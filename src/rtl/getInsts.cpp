#include "GetInsts.h"
#include "Port.h"
#include "Cell.h"
#include "Module.h"

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
        re2::StringPiece input(value);
        std::string s,t;
        if (regexp && re2::RE2::FullMatch(input, regex, &s, &t)) {
            return true;
        }
    }
    return false;
}

void rtl::getInsts(std::vector<Inst*>* insts, std::vector<instFilter>& filters, Referable<Inst>* inst)
{
    for (auto& filter : filters) {
        if (filter.blackbox && !inst->cell_ref->module_ref->blackbox) {
            continue;
        }

        if (filter.regexp && filter.name.length() && !filter.name_regex.get()) {
            filter.name_regex.reset(new re2::RE2(filter.name));
        }
        if (filter.regexp && filter.port_name.length() && !filter.port_regex.get()) {
            filter.port_regex.reset(new re2::RE2(filter.port_name));
        }
        if (filter.regexp && filter.cell_name.length() && !filter.cell_regex.get()) {
            filter.cell_regex.reset(new re2::RE2(filter.cell_name));
        }
        if (filter.regexp && filter.cell_type.length() && !filter.type_regex.get()) {
            filter.type_regex.reset(new re2::RE2(filter.cell_type));
        }

        bool found = true;
        std::string inst_name = inst->makeName();
        if (filter.name.length() && !compare(inst_name, filter.name, filter.partial, filter.regexp, *filter.name_regex.get())) {
            found = false;
        }
        if (filter.cell_name.length() && !compare(inst->cell_ref->name, filter.cell_name, filter.partial, filter.regexp, *filter.cell_regex.get())) {
            found = false;
        }
        if (filter.cell_type.length() && !compare(inst->cell_ref->type, filter.cell_type, filter.partial, filter.regexp, *filter.type_regex.get())) {
            found = false;
        }
        bool found_port = false;
        if (filter.port_name.length())
        {
            for (auto& conn : inst->conns) {
                std::string port_name = conn.makeName(&inst_name);
                if (compare(port_name, filter.port_name, filter.partial, filter.regexp, *filter.port_regex.get())) {
                    found_port = true;
                }
            }
            if (!found_port) {
                found = false;
            }
        }
        if (found) {
            insts->push_back(inst);
        }
    }

    for (auto& sub_inst : inst->insts) {
        getInsts(insts, filters, &sub_inst);
    }
}
