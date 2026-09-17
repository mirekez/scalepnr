#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>

#include "NodeMask.h"

namespace fpga {

// Cache immutable endpoint queries for one placement operation, never leases.
// Database/model edits must occur outside the scope; destruction frees all entries.
class PinLookupCache
{
public:
    enum class Query : uint8_t { ResourcePin, TilePin, RoutedTilePin };
    struct Key {
        const void* model;
        uint8_t direction;
        std::string pin;
        int site;
        std::string route_type;
        bool strict;
        bool has_delta;
        int dx;
        int dy;
        std::string cell_type;
        const void* crossbar = nullptr;
        Query query = Query::ResourcePin;

        bool operator==(const Key&) const = default;
    };

private:
    struct Hash {
        // Include every filter used by the reference endpoint lookup.
        size_t operator()(const Key& key) const {
            size_t hash = std::hash<const void*>{}(key.model);
            auto combine = [&](size_t value) {
                hash ^= value + 0x9e3779b9U + (hash << 6) + (hash >> 2);
            };
            combine(key.direction);
            combine(std::hash<std::string>{}(key.pin));
            combine(std::hash<int>{}(key.site));
            combine(std::hash<std::string>{}(key.route_type));
            combine(key.strict);
            combine(key.has_delta);
            combine(std::hash<int>{}(key.dx));
            combine(std::hash<int>{}(key.dy));
            combine(std::hash<std::string>{}(key.cell_type));
            combine(std::hash<const void*>{}(key.crossbar));
            combine(static_cast<uint8_t>(key.query));
            return hash;
        }
    };
    inline static thread_local PinLookupCache* current = nullptr;
    PinLookupCache* previous;
    size_t limit;
    std::unordered_map<Key, NodeMask, Hash> entries;

public:
    size_t hits = 0;
    size_t misses = 0;

    // Bound scratch memory and restore the caller's cache when scopes nest.
    explicit PinLookupCache(size_t max_entries = 16384)
        : previous(current), limit(max_entries) { current = this; }
    ~PinLookupCache() { current = previous; }
    PinLookupCache(const PinLookupCache&) = delete;
    PinLookupCache& operator=(const PinLookupCache&) = delete;

    static PinLookupCache* active() { return current; }
    size_t size() const { return entries.size(); }

    // Cache misses, including empty masks; a full cache never changes semantics.
    template<typename Compute>
    NodeMask resolve(Key key, Compute&& compute) {
        auto found = entries.find(key);
        if (found != entries.end()) {
            ++hits;
            return found->second;
        }
        ++misses;
        NodeMask result = compute();
        if (entries.size() < limit) entries.emplace(std::move(key), result);
        return result;
    }
};

}
