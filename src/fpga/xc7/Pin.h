#pragma once

namespace fpga {

struct Pin
{
    std::string name;
    std::string bank;
    std::string site;
    std::string tile;
    std::string function;
    Coord pos;
};

}
