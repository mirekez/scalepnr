#pragma once

#include "referable.h"

namespace gear
{

struct Module;

struct Port: public Referable
{
    enum {
      PORT_IN,
      PORT_OUT,
      PORT_BIDIR,
    };

    Ref<Module> module;
};



}
