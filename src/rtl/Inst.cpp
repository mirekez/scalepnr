#include "Inst.h"

using namespace rtl;

std::string Inst::makeName(size_t limit)
{
    std::string name = depth == 0 ? "" : cell_ref->name;
    name.reserve(128);
    Referable<Inst>* inst = parent_ref.get();
    if (inst)
    while (inst->parent_ref.get() != 0) {
        name.insert(0, "|");
        name.insert(0, inst->cell_ref->name);
        inst = inst->parent_ref.get();
    }
    return shortenName(name, limit);
}

Conn* Inst::operator [](const std::string& port_name)
{
    for (auto& conn : conns) {
        if (conn.port_ref->name == port_name) {
            return &conn;
        }
    }
    return nullptr;
}

long Inst::mark_counter = 0;
