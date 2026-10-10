#pragma once

#include "Device.h"
#include <algorithm>
#include <bit>

namespace pnr {

// A placement preference, not a routability proof or a new packing rule.
// Read live leases: placement neither changes routes nor needs an owner search.
struct PlaceRouting {
    bool enabled = false;
    // Only immutable topology counts are reused. Occupancy is always live;
    // aliases of a numeric node must not inflate its bank's capacity.
    mutable std::unordered_map<const fpga::CBType*, std::array<size_t, 3>> bank_sizes;

    void reset() { bank_sizes.clear(); }

    double pressure(Coord coord) const {
        if (!enabled) return 0;
        auto& device = fpga::Device::current();
        if (coord.x < 0 || coord.y < 0 || coord.x >= device.size_width
            || coord.y >= device.size_height) return 0;
        auto* route = device.routeTile(device.tile_grid[coord.y*device.size_width + coord.x]);
        if (!route || !route->cb_type) return 0;
        auto [entry, inserted] = bank_sizes.try_emplace(route->cb_type);
        if (inserted) {
            for (const auto& [node, name] : route->cb_type->node_names) {
                if (node.type == fpga::CB_NODE_SRC) ++entry->second[0];
                if (node.type == fpga::CB_NODE_DST) ++entry->second[1];
                if (node.type == fpga::CB_NODE_JOINT) ++entry->second[2];
            }
        }
        auto fraction = [](const NodeMask& occupied, size_t capacity) {
            size_t used = 0;
            for (auto word : occupied.words) {
                used += std::popcount(static_cast<uint64_t>(word));
                used += std::popcount(static_cast<uint64_t>(word >> 64));
            }
            // Unnamed numeric fixtures still report occupied fabric. Empty,
            // unknown fabrics do not invent capacity or prohibit placement.
            return used ? std::min(1.0, double(used)/std::max<size_t>(1, capacity)) : 0.0;
        };
        // Do not hide a saturated small JOINT bank in a large SRC/DST bank.
        return std::max({fraction(route->cb.src.jump, entry->second[0]),
                         fraction(route->cb.dst.jump, entry->second[1]),
                         fraction(route->cb.joint.jump, entry->second[2])});
    }

    bool allowsMove(Coord from, Coord to) const {
        return !enabled || pressure(to) <= pressure(from) + 1e-12;
    }
};
}
