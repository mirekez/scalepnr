#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "Element.h"

namespace rtl { struct Inst; struct Conn; }

namespace fpga {
struct Tile;
struct TileType;
struct CBType;

// Scratch mappings for one bunch search. Models, pins and net connectivity must
// stay unchanged within the scope; cell assignments and input owners remain live.
class PackingPinMappings
{
    struct Key {
        const TileType* model;
        const CBType* resource_cb;
        const CBType* route_cb;
        const rtl::Inst* inst;
        int dx;
        int dy;
        uint16_t positions;
        bool operator==(const Key&) const = default;
    };
    struct Hash {
        size_t operator()(const Key& key) const {
            size_t hash = std::hash<const void*>{}(key.model);
            auto combine = [&](size_t value) {
                hash ^= value + 0x9e3779b9U + (hash << 6) + (hash >> 2);
            };
            combine(std::hash<const void*>{}(key.resource_cb));
            combine(std::hash<const void*>{}(key.route_cb));
            combine(std::hash<const void*>{}(key.inst));
            combine(std::hash<int>{}(key.dx));
            combine(std::hash<int>{}(key.dy));
            combine(key.positions);
            return hash;
        }
    };
    struct Input {
        rtl::Conn* driver;
        uint16_t local;
        uint16_t positions;
    };
    size_t limit;
    std::unordered_map<Key, std::vector<Input>, Hash> entries;

public:
    size_t hits = 0;
    size_t builds = 0;
    explicit PackingPinMappings(size_t max_entries = 256) : limit(max_entries) {}
    size_t size() const { return entries.size(); }
    // Resolve each model/instance once, then filter by numeric IDs and live ownership.
    uint16_t filter(Tile& tile, Tile& route_tile, rtl::Inst& inst,
                    ElementType type, uint16_t available);
};
}
