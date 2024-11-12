#pragma once

#include "Module.h"
#include "Conn.h"
#include "referable.h"

#include <vector>
#include <list>

namespace rtl
{

struct Cell
{
    std::string name;
    std::string type;
    Ref<Module> module_ref;
    std::vector<Referable<Port>> ports;
};

struct CellInst
{
    // must have
    int depth = -1;
    int height = -1;
    Ref<Cell> cell_ref;
    // optional
    Ref<CellInst> parent_ref;  // can be zero for top
    std::vector<Referable<Conn>> conns;
    std::list<Referable<CellInst>> insts;

    std::string makeName() {
        std::string name = cell_ref->name;
        name.reserve(128);
        Referable<CellInst>* inst = parent_ref.get();
        while (inst->parent_ref.get() != 0) {
            name.insert(0, "|");
            name.insert(0, inst->cell_ref->name);
            inst = inst->parent_ref.get();
        }
        return name;
    }
};



}
