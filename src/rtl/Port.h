#pragma once

#include "referable.h"

namespace rtl
{

struct Module;

struct Port
{
    // must have
    std::string name;
    int designator = -1;
    int bitnum = 0;
    enum {
      PORT_IN,
      PORT_OUT,
      PORT_IO,
    } type = PORT_IO;

    void setType(const std::string& type_str)
    {
        type = type_str == "input" ? PORT_IN : (type_str == "output" ? PORT_OUT : PORT_IO );
    }

    std::string getType()
    {
        return type == PORT_IN ? "input" : (type == PORT_OUT ? "output" : "inout" );
    }

    char getTypeChar()
    {
        return type == PORT_IN ? '\\' : (type == PORT_OUT ? '/' : '~');
    }

    bool special = false;
    // optional
//    Ref<Module> module;
};



}
