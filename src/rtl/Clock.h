#pragma once

#include "Conn.h"

#include <string>

namespace rtl
{

struct Clock
{
    // must have
    std::string name;
    Referable<Conn>* conn_ptr = nullptr;
    std::string conn_name;
    double period_ns = 0;
    int duty = 50;
};




}
