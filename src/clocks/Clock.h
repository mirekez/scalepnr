#pragma once

#include "Conn.h"

#include <string>

namespace clk
{

struct Clock
{
    // must have
    std::string name;
    Referable<rtl::Conn>* conn_ptr;
    std::string conn_name;
    double period_ns;
    int duty;
};




}

