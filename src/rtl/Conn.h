#pragma once

#include "referable.h"

#include <string>

namespace rtl
{

struct Inst;
struct Port;


struct Conn: public Ref<Conn>  // Conn contains reference to other Conn, there must be no other Ref<Conn> in the whole project !!! (because we are considering all peers as connections)
{
    // must have
    Ref<Port> port_ref;
    Ref<Inst> inst_ref;

    // optional
    std::string makeName(std::string* inst_name_hint = 0, size_t limit = 250);
    std::string makeNetName(std::string* inst_name_hint = 0, size_t limit = 250);

    Referable<Conn>* operator ->() = delete;  // we dont want to use operator -> from Ref<Conn> to prevent bugs
    static Referable<Conn>& fromRef(Ref<Conn>& peer)
    {
        return static_cast<Referable<Conn>&>(peer);  // please, no other class can have Ref<Conn>, except this. It makes this casting guaranteed
    }

    Conn* follow();
};




}
