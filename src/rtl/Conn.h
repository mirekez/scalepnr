#pragma once

#include "referable.h"

#include <string>

namespace rtl
{

struct Inst;
struct Port;

struct Conn: public Ref<Conn>
{
    // must have
    Ref<Port> port_ref;
    Ref<Inst> inst_ref;
    // optional
//     output_ref;  // can be zero for some time before linkage

    std::string makeName(std::string* inst_name_hint = 0);
};


}
