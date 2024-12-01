#pragma once

#include "Module.h"
#include "Inst.h"
#include "Conn.h"
#include "Cell.h"
#include "Timing.h"
#include "referable.h"

#include <vector>
#include <list>

inline std::string shortenName(const std::string& in, size_t limit)
{
    if (in.length() <= limit) {
        return in;
    }
    return in.substr(0,limit/3) + "..." + in.substr(in.length()-limit/3,limit/3);
}

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
    bool locked = false;  // for traversal locks
    Ref<Timing> timing;  // used in Timings engine, no other links from here

    std::string makeName(size_t limit = 100)
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
};



}
