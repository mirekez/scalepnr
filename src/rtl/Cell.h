#pragma once

#include "Module.h"
#include "Port.h"
#include "referable.h"

#include <vector>

namespace rtl
{

struct Cell
{
    // must have
    std::string name;
    std::string type;
    Ref<Module> module_ref;
    std::vector<Referable<Port>> ports;  // upper connections between neighbour cells and module ports
    // optional
    std::vector<double> latency_matrix;  // inputs x outputs delays in ns
};


}
