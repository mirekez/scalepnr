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
    Ref<Port> port_ref;
    Ref<CellInst> inst_ref;
    Ref<Conn> output_ref;
    int designator;
};



}