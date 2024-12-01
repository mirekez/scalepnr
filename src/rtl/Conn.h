#pragma once

#include "referable.h"

#include <string>

namespace rtl
{

struct Inst;
struct Port;

struct Conn: public Ref<Conn>  // Conn begins with reference to other Conn, can be zero
{
    // must have
    Ref<Port> port_ref;
    Ref<Inst> inst_ref;

    // optional
    std::string makeName(std::string* inst_name_hint = 0, size_t limit = 100);
    std::string makeNetName(std::string* inst_name_hint = 0, size_t limit = 100);

    Referable<Conn>* operator ->() = delete;  // we dont want to use operator -> from Ref<Conn> to prevent bugs
};


}
