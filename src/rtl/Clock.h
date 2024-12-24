#pragma once

#include "Conn.h"

#include <string>

namespace rtl
{

struct Clock
{
    // must have
    std::string name;
    Referable<Conn>* conn_ptr;
    std::string conn_name;
    double period_ns;
    int duty;
    // optional
    Referable<Inst>* bufg_ptr = nullptr;
};




}

