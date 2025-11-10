#pragma once

namespace fpga {

struct Pin
{
    std::string name;
    std::string bank;
    std::string tile;
    std::string function;
    Coord pos;
};

}
