#pragma once

#include "Module.h"
#include "Port.h"
#include "referable.h"

#include <vector>

namespace rtl
{

struct Cell
{
    std::string name;
    std::string type;
    Ref<Module> module_ref;
    std::vector<Referable<Port>> ports;  // upper connections to neighbour cells and to module ports
};


}
