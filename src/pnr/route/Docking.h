#pragma once

#include "Tile.h"
#include "Wire.h"
#include "NodeMask.h"

#include <vector>

namespace pnr {

struct DockingResult
{
    bool success = false;
    std::vector<fpga::Wire> fragments;
    int target_seed_count = 0;
    int target_entry_count = 0;
    int target_busy_count = 0;
    int forward_push_count = 0;
    int backward_push_count = 0;
    int forward_pop_count = 0;
    int backward_pop_count = 0;
};

// Bidirectional grounding fallback: connect a routed forward frontier to one
// destination-entry rail by exploring only loaded crossbar masks and jump deltas.
DockingResult dockGrounding(fpga::Tile& forward_tile, int forward_dst,
                            const std::string& forward_dst_wire,
                            fpga::Tile& target_tile, NodeMask pin_nodes,
                            int max_depth = 5, int radius = 5);

// I/O endpoint docking uses the same mask-only transitions with a wider
// edge-interface window, expanded as a direction-led beam toward the endpoint.
DockingResult dockIOB(fpga::Tile& forward_tile, int forward_dst,
                      const std::string& forward_dst_wire,
                      fpga::Tile& target_tile, NodeMask pin_nodes);

}
