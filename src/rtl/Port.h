#pragma once

#include "referable.h"

namespace rtl
{

struct Module;

struct Port
{
    std::string name;
    enum {
      PORT_IN,
      PORT_OUT,
      PORT_IO,
    } type;
    int designator;
//    Ref<Module> module;
};



}
