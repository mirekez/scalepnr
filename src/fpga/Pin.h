#pragma once

#include <cstdint>
#include <string>

namespace fpga {

struct Pin
{
    enum Direction : int8_t {
        PIN_UNKNOWN = -1,
        PIN_INPUT = 0,
        PIN_OUTPUT = 1,
        PIN_INOUT = 2,
    };

    std::string name;
    std::string bank;
    std::string site;
    std::string tile;
    std::string function;
    Coord pos;
    // Optional tile/site pin fields are populated by tile-type databases.
    std::string port;
    std::string wire;
    int site_pos = -1;
    Direction direction = PIN_UNKNOWN;
};

}
