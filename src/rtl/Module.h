#pragma once

#include "Port.h"
#include "referable.h"

namespace gear
{

class Module: public Referable
{
    std::string name;
    std::vector<Port> ports;
    std::vector<Ref<Module>> submodules;
};





}
