#pragma once

#include "referable.h"

namespace rtl
{

struct Module;

struct Port
{
    std::string name;
    int width = -1;
    enum {
      PORT_IN,
      PORT_OUT,
      PORT_IO,
    } type;
    std::vector<int> designators;
    Ref<Module> module;
};



}
