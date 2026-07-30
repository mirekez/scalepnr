#pragma once

#include "Tile.h"
#include "Wire.h"
#include "NodeMask.h"

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace fpga {
struct Device;
}

namespace pnr {

struct BackwardResolveKey
{
    int x = 0;
    int y = 0;
    int dst = -1;

    bool operator<(const BackwardResolveKey& other) const
    {
        if (x != other.x) {
            return x < other.x;
        }
        if (y != other.y) {
            return y < other.y;
        }
        return dst < other.dst;
    }
};

struct BackwardResolveSource
{
    fpga::Tile* tile = nullptr;
    int src = -1;
    int route_jump = -1;
};

struct BackwardResolveIndex
{
    std::map<BackwardResolveKey, std::vector<BackwardResolveSource>> sources;
    int mapping_scan_count = 0;
    int mapping_reaches_count = 0;
};

// Resolve every numeric SRC mapping in a bounded grid region and index it by
// the destination tile and DST node reached by that source.
BackwardResolveIndex buildBackwardResolveIndex(
    fpga::Device& device, fpga::Coord center, int radius,
    const std::function<bool(const fpga::Coord&)>& include_source = {});

struct DockingBackwardAttempt
{
    int target_dst = -1;
    std::string result;
    std::vector<fpga::Wire> fragments;
};

struct DockingResult
{
    bool success = false;
    std::vector<fpga::Wire> fragments;
    bool blocked_terminal_reachable = false;
    int blocked_dst = -1;
    int blocked_pin = -1;
    int blocked_joint = -1;
    int blocked_joint2 = -1;
    int target_seed_count = 0;
    int target_entry_count = 0;
    int target_busy_count = 0;
    int forward_push_count = 0;
    int backward_push_count = 0;
    int forward_pop_count = 0;
    int backward_pop_count = 0;
    int backward_mapping_scan_count = 0;
    int backward_mapping_reaches_count = 0;
    int backward_missing_prev_dst_count = 0;
    int backward_topology_reject_count = 0;
    int backward_busy_reject_count = 0;
    int backward_seen_reject_count = 0;
    int backward_deadend_count = 0;
    int backward_deadend_reject_count = 0;
    std::vector<DockingBackwardAttempt> backward_attempts;
};

// Bidirectional grounding fallback: connect a routed forward frontier to one
// destination-entry rail by exploring only loaded crossbar masks and jump deltas.
DockingResult dockGrounding(fpga::Tile& forward_tile, int forward_dst,
                            const std::string& forward_dst_wire,
                            fpga::Tile& target_tile, NodeMask pin_nodes,
                            int max_depth = 5, int radius = 5,
                            bool trace_backward_attempts = false);

// I/O endpoint docking uses the same mask-only transitions with a wider
// edge-interface window, expanded as a direction-led beam toward the endpoint.
DockingResult dockIOB(fpga::Tile& forward_tile, int forward_dst,
                      const std::string& forward_dst_wire,
                      fpga::Tile& target_tile, NodeMask pin_nodes,
                      bool trace_backward_attempts = false);

}
