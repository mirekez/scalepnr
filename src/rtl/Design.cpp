#include "Design.h"

using namespace rtl;

Design& Design::current()
{
    static Design current;
    return current;
}

bool Design::getInsts(std::vector<Inst*>* insts, std::string name, std::string port_name, std::string cell_name, bool partial_name, Referable<Inst>* inst)
{
    if (!inst) {
        inst = &top;
    }

    std::string inst_name = inst->makeName();
    if (inst_name == name || (name.length() && partial_name && inst_name.find(name) != std::string::npos)) {
        insts->push_back(inst);
    }
    else
    if (inst->cell_ref->name == cell_name || (cell_name.length() && partial_name && inst->cell_ref->name.find(cell_name) != std::string::npos)) {
        insts->push_back(inst);
    }
    else
    for (auto& conn : inst->conns) {
        std::string port = inst_name + "." + conn.port_ref->name;
        if (port_name == port || (port_name.length() && partial_name && port_name.find(name) != std::string::npos)) {
            insts->push_back(inst);
            break;
        }
    }

    for (auto& sub_inst : inst->insts) {
        if (!getInsts(insts, name, port_name, cell_name, partial_name, &sub_inst)) {
            return false;
        }
    }
    return true;
}

