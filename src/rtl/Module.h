#pragma once

#include "Port.h"
#include "referable.h"

namespace rtl
{

struct Module
{
    std::string name;
    std::vector<Port> ports;
    std::vector<Ref<Module>> submodules;
};





}
