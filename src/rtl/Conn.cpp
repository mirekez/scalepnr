#include "Conn.h"
#include "Inst.h"
#include "debug.h"

#include <format>

using namespace rtl;

std::string Conn::makeName(std::string* inst_name_hint, size_t limit)
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
    return shortenName(inst_name, limit/2) + shortenName(port_ref->makeName(), limit/2);
}

std::string Conn::makeNetName(std::string* inst_name_hint, size_t limit)
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
    Module* parent = inst_ref->cell_ref->module_ref->parent_ref.peer;
    if (parent)
    for (auto& net : parent->nets) {
        for (size_t i=0; i < net.designators.size(); ++i) {
            if (net.designators[i] == designator) {
                net_name = std::format("{}[{}]", net.name, i);
            }
        }
    }

    return shortenName(inst_name, limit/2) + shortenName(port_ref->makeName(), limit/2);
}

Conn* Conn::follow()
{
    Conn* curr = this;
    for (Conn* conn = this; conn; conn = conn->peer) {
        PNR_LOG3("CONN", " -> '{}'('{}')", conn->makeName(), conn->inst_ref->cell_ref->type);
        curr = conn;
    }
    return curr==this?nullptr:curr;
}
