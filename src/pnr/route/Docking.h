#pragma once

#include "Tile.h"
#include "Wire.h"
#include "u256.h"

#include <vector>

namespace pnr {

struct DockingResult
{
    bool success = false;
    std::vector<fpga::Wire> fragments;
};

// Bidirectional grounding fallback: connect a routed forward frontier to one
// destination-entry rail by exploring only abstract CB masks and jump geometry.
DockingResult dockGrounding(fpga::Tile& forward_tile, int forward_dst,
                            const std::string& forward_dst_wire,
                            fpga::Tile& target_tile, u256 pin_nodes,
                            int max_depth = 5, int radius = 5);

}
