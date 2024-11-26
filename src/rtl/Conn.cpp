#include "Conn.h"
#include "Inst.h"

#include <format>
#include <print>

using namespace rtl;

std::string Conn::makeName(std::string* inst_name_hint)
{
    std::string inst_name;
    if (!inst_name_hint) {
        inst_name = inst_ref->makeName();
    }
    else {
        inst_name = *inst_name_hint;
    }
    if (inst_name.length()) {
        inst_name += ".";
    }
    return inst_name + port_ref->makeName();
}

std::string Conn::makeNetName(std::string* inst_name_hint)
{
    std::string inst_name;
    if (!inst_name_hint) {
        inst_name = inst_ref->makeName();
    }
    else {
        inst_name = *inst_name_hint;
    }
    if (inst_name.length()) {
        inst_name += ".";
    }

    std::string net_name = port_ref->name;
    int designator = port_ref->designator;
    Module* parent = inst_ref->cell_ref->module_ref->parent_ref.ref;
    if (parent)
    for (auto& net : parent->nets) {
        for (size_t i=0; i < net.designators.size(); ++i) {
            if (net.designators[i] == designator) {
                net_name = std::format("{}[{}]", net.name, i);
            }
        }
    }

    return inst_name + port_ref->makeName();
}

