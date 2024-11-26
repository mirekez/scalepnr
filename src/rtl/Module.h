#pragma once

#include "Port.h"
#include "Cell.h"
#include "Net.h"
#include "referable.h"

namespace rtl
{

struct Module
{
    // must have
    std::string name;
    bool blackbox = true;
    // optional
    std::vector<Referable<Cell>> cells;
    std::vector<Net> nets;  // we are almost not using it
    std::vector<Port> interface;  // using only to prepare netlist in memory
    std::vector<Ref<Module>> submodules_ref;
    Ref<Module> parent_ref;  // can be 0 in top module
};





}
