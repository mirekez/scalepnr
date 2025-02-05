#pragma once

#include "referable.h"
#include "Port.h"
#include "debug.h"

#include <string>

namespace rtl
{

struct Inst;  // Inst uses Conn

struct Conn: public Ref<Conn>  // Conn contains reference to other Conn, there must be no other Ref<Conn> in the whole project !!! (because we are considering all peers as connections)
{
    // must have
    Ref<Port> port_ref;
    Ref<Inst> inst_ref;

    // optional
    std::string makeName(std::string* inst_name_hint = 0, size_t limit = 250);
    std::string makeNetName(std::string* inst_name_hint = 0, size_t limit = 250);

    Referable<Conn>* operator ->() = delete;  // we dont want to use operator -> from Ref<Conn> to prevent bugs

    void check();

    template<class Base>
    static Referable<Conn>& fromBase(Base& base)
    {
        return static_cast<Referable<Conn>&>(  // no other class can have Ref<Conn>, except this. It makes this casting guaranteed
                static_cast<Ref<Conn>&>(base)  // just give RefBase<Conn> here to let convert it to Ref<Conn>
            );
    }

    template<class Base>
    static auto& getSinks(Base& base)  // Referable<Conn> has 'peer' and 'peers'. We use 'peer' for inputs/bidir and 'peers' for outputs
    {
        PNR_ASSERT(fromBase(base).port_ref->type == Port::PORT_OUT, "sinks of input port requested");
        PNR_ASSERT(fromBase(base).peer == nullptr, "output port input connection is not zero");
        return fromBase(base).peers;
    }

    Conn* follow();
};




}
