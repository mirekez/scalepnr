#pragma once

#include "referable.h"

namespace rtl
{

struct Port;
struct Cell;
struct Conn;

struct Conn
{
    std::string name;
    enum {
      CONN_IN,
      CONN_OUT,
      CONN_IO,
    } type;

    Ref<Port> port;
    Ref<Cell> cell;
    Ref<Conn> conn;
};



}
