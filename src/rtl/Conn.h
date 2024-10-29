#pragma once

#include "referable.h"

namespace gear
{

struct Port;
struct Cell;
struct Conn;

struct Conn: public Referable
{
    Ref<Port> port;
    Ref<Cell> cell;
    Ref<Conn> conn;
};



}
