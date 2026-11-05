#pragma once

#include "Tile.h"
#include "Wire.h"

#include <array>
#include <unordered_map>

namespace pnr {

// One exhausted, driver-scoped numeric region. A bitmap per typed tile avoids
// retaining a hash entry per expanded node. This is not a path or a lease.
struct InputRouteRegionCache {
    using Nodes=std::unordered_map<fpga::Tile*,std::array<NodeMask,5>>;
    static constexpr size_t max_tile_maps=8192;
    Nodes reached, boundary;
    fpga::Coord center{};
    int radius=-1;
    uint64_t revision=0;
    bool reuse_allowed=false;
    size_t hits=0;

    static bool contains(const Nodes& nodes,fpga::Tile& tile,fpga::CBNodeNameType kind,int node) {
        auto found=nodes.find(&tile);
        return node>=0 && node<CB_MAX_NODES && kind<5 && found!=nodes.end() &&
               found->second[kind].testBit(node);
    }
    void clear() { reached.clear();boundary.clear();radius=-1;reuse_allowed=false; }
};

} // namespace pnr
