#pragma once

#include "Port.h"
#include "Cell.h"
#include "referable.h"

namespace rtl
{

struct Module
{
    // must have
    std::string name;
    bool blackbox = true;
    // optional
    std::vector<Referable<Port>> ports;
    std::vector<Referable<Cell>> cells;
    std::vector<Ref<Module>> submodules_ref;
};





}
