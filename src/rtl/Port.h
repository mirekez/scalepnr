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
    enum {
      PORT_IN,
      PORT_OUT,
      PORT_IO,
    } type = PORT_IO;
    int index = -1;  // port number, I/O separately
    int bitnum = -1;
    int designator = -1;

    bool is_global = false;

    // optional
    int sub_designator = -1;  // designator to connect cell's external ports to internal subcells's ports (taken from module ports)

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
