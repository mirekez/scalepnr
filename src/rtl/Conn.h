#pragma once

#include "referable.h"

#include <vector>

namespace rtl
{

struct Port;
struct Conn;
struct CellInst;

struct Conn
{
    // must have
    Ref<Port> port_ref;
    Ref<CellInst> inst_ref;
    // optional
    Ref<Conn> output_ref;  // can be zero for some time before linkadge
};


}
