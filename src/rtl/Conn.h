#pragma once

#include "referable.h"

#include <vector>

namespace rtl
{

struct Port;
struct Conn;
struct Inst;

struct Conn: public Ref<Conn>
{
    // must have
    Ref<Port> port_ref;
    Ref<Inst> inst_ref;
    // optional
//     output_ref;  // can be zero for some time before linkage
};


}
