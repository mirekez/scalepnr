#pragma once

#include "Conn.h"

#include <string>

namespace clocks
{

struct Clock
{
    // must have
    std::string name;
    rtl::Conn* conn_ptr;
    std::string conn_name;
    double period_ns;
    int duty;
};




}

