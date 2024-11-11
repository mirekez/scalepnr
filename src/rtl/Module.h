#pragma once

#include "Port.h"
#include "Cell.h"
#include "referable.h"

namespace rtl
{

struct Module
{
    std::string name;
    std::vector<Referable<Port>> ports;
    std::vector<Referable<Cell>> cells;
    std::vector<Ref<Module>> submodules_ref;
};





}
