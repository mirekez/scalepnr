#pragma once

#include "referable.h"

#include <string>
#include <format>

namespace rtl
{

struct Module;

struct Port
{
    // must have
    std::string name;
    int bitnum = -1;
    int designator = -1;
    enum {
      PORT_IN,
      PORT_OUT,
      PORT_IO,
    } type = PORT_IO;

    bool global = false;
    // optional
    int sub_designator = -1;  // designator to connect cell's external ports to internal subcells's ports (taken from module ports)

    void setType(const std::string& type_str)
    {
        type = type_str == "input" ? PORT_IN : (type_str == "output" ? PORT_OUT : PORT_IO );
    }

    std::string getType()
    {
        return type == PORT_IN ? "input" : (type == PORT_OUT ? "output" : "inout" );
    }

    const char* getTypeChar()
    {
        return type == PORT_IN ? "I" : (type == PORT_OUT ? "O" : "IO");
    }

    std::string makeName()
    {
        return name + (bitnum==-1?"":std::format("[{}]", bitnum));
    }
};



}
