#pragma once

#include "Module.h"
#include "Inst.h"
#include "Conn.h"
#include "Cell.h"
#include "referable.h"

#include <vector>
#include <list>

namespace rtl
{

struct Inst
{
    // must have
    int depth = -1;
    int height = -1;
    Ref<Cell> cell_ref;
    // optional
    Ref<Inst> parent_ref;  // can be zero for top
    std::vector<Referable<Conn>> conns;
    std::list<Referable<Inst>> insts;

    std::string makeName()
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
        return name;
    }
};



}
