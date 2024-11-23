#pragma once

#include "referable.h"

#include <string>

namespace rtl
{

struct Inst;
struct Port;

struct Conn: public Ref<Conn>  // can be zero for some time before linkage
{
    // must have
    Ref<Port> port_ref;
    Ref<Inst> inst_ref;
    // optional

    std::string makeName(std::string* inst_name_hint = 0);
};


}
