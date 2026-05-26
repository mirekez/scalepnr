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
    // Optional tile/site pin fields are populated by tile-type databases.
    std::string port;
    std::string wire;
    int site_pos = -1;
};

}
