#pragma once

#include "referable.h"

namespace rtl
{

struct Port;
struct Cell;
struct Conn;

struct Conn
{
    Ref<Port> port;
    Ref<Cell> cell;
    Ref<Conn> conn;
};



}
