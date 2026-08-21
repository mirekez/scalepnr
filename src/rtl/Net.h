#pragma once

#include "referable.h"
#include "Port.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace fpga {
struct Wire;
}

namespace rtl
{

struct Inst;

struct NetRouteBinding
{
    Inst* owner = nullptr;
    size_t route_index = std::numeric_limits<size_t>::max();
    Inst* from = nullptr;
    Inst* to = nullptr;
    std::string from_port;
    std::string to_port;
    std::string route_name;
    uint64_t route_id = 0;
};

struct NetRouteLookup
{
    size_t index = std::numeric_limits<size_t>::max();
    size_t count = 0;
};

struct Net
{
    // optional
    std::string name;
    std::vector<int> designators;

    Ref<fpga::Wire> wire;
    Ref<Port> src_port;
    Ref<Port> dst_port;
    bool void_net = false;
    // Infrastructure nets may reserve routing resources before ordinary routing.
    bool route_protected = false;
    // Distributed sources start independently from database-declared local
    // nodes instead of one placed resource endpoint.
    bool distributed_source = false;
    // Select which of the two database-declared distributed capabilities owns
    // this net; this is source identity and survives scheduler reconstruction.
    bool distributed_one = true;
    std::vector<int> void_designators;
    std::vector<NetRouteBinding> routes;

private:
    struct RouteLookupEntry
    {
        size_t index = std::numeric_limits<size_t>::max();
        size_t count = 0;
    };

    mutable std::unordered_map<size_t, RouteLookupEntry> route_lookup;
    mutable size_t route_lookup_size = std::numeric_limits<size_t>::max();
    mutable const NetRouteBinding* route_lookup_data = nullptr;
    mutable std::unordered_map<uint64_t, size_t> route_id_lookup;
    mutable size_t route_id_lookup_size = std::numeric_limits<size_t>::max();
    mutable const NetRouteBinding* route_id_lookup_data = nullptr;
    uint64_t next_route_id = 1;

    static size_t routeLookupHash(const Inst* from, const Inst* to,
                                  const std::string& from_port,
                                  const std::string& to_port,
                                  const std::string& route_name)
    {
        size_t hash = std::hash<const void*>{}(from);
        auto combine = [&](size_t value) {
            hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
        };
        combine(std::hash<const void*>{}(to));
        combine(std::hash<std::string>{}(from_port));
        combine(std::hash<std::string>{}(to_port));
        combine(std::hash<std::string>{}(route_name));
        return hash;
    }

    static bool routeBindingMatches(const NetRouteBinding& binding,
                                    const Inst* from, const Inst* to,
                                    const std::string& from_port,
                                    const std::string& to_port,
                                    const std::string& route_name)
    {
        return binding.from == from && binding.to == to
            && binding.from_port == from_port && binding.to_port == to_port
            && binding.route_name == route_name;
    }

    bool routeLookupCurrent() const
    {
        return route_lookup_size == routes.size()
            && route_lookup_data == routes.data();
    }

    bool routeIdLookupCurrent() const
    {
        return route_id_lookup_size == routes.size()
            && route_id_lookup_data == routes.data();
    }

    void rebuildRouteLookup() const
    {
        route_lookup.clear();
        route_lookup.reserve(routes.size());
        for (size_t index = 0; index < routes.size(); ++index) {
            const NetRouteBinding& binding = routes[index];
            size_t hash = routeLookupHash(binding.from, binding.to,
                binding.from_port, binding.to_port, binding.route_name);
            auto [it, inserted] = route_lookup.emplace(
                hash, RouteLookupEntry{index, 1});
            if (!inserted) {
                ++it->second.count;
            }
        }
        route_lookup_size = routes.size();
        route_lookup_data = routes.data();
    }

public:
    // Find one exact physical endpoint binding in expected constant time.
    // count greater than one preserves the caller's duplicate-binding handling.
    NetRouteLookup findRouteBinding(const Inst* from, const Inst* to,
                                    const std::string& from_port,
                                    const std::string& to_port,
                                    const std::string& route_name) const
    {
        if (!routeLookupCurrent()) {
            rebuildRouteLookup();
        }
        size_t hash = routeLookupHash(from, to, from_port, to_port, route_name);
        auto found = route_lookup.find(hash);
        if (found == route_lookup.end()) {
            return {};
        }
        const RouteLookupEntry& entry = found->second;
        if (entry.count == 1 && entry.index < routes.size()
            && routeBindingMatches(routes[entry.index], from, to, from_port,
                                   to_port, route_name)) {
            return {entry.index, 1};
        }
        size_t first = std::numeric_limits<size_t>::max();
        size_t count = 0;
        for (size_t index = 0; index < routes.size(); ++index) {
            if (!routeBindingMatches(routes[index], from, to, from_port,
                                     to_port, route_name)) {
                continue;
            }
            if (first == std::numeric_limits<size_t>::max()) {
                first = index;
            }
            ++count;
        }
        return {first, count};
    }

    void appendRouteBinding(NetRouteBinding binding)
    {
        if (binding.route_id == 0) {
            binding.route_id = next_route_id++;
        } else {
            next_route_id = std::max(next_route_id, binding.route_id + 1);
        }
        bool lookup_current = routeLookupCurrent();
        bool id_lookup_current = routeIdLookupCurrent();
        uint64_t route_id = binding.route_id;
        size_t hash = routeLookupHash(binding.from, binding.to,
            binding.from_port, binding.to_port, binding.route_name);
        size_t index = routes.size();
        routes.push_back(std::move(binding));
        if (id_lookup_current) {
            route_id_lookup[route_id] = index;
            route_id_lookup_size = routes.size();
            route_id_lookup_data = routes.data();
        } else {
            invalidateRouteIdLookup();
        }
        if (!lookup_current) {
            invalidateRouteLookup();
            return;
        }
        auto [it, inserted] = route_lookup.emplace(
            hash, RouteLookupEntry{index, 1});
        if (!inserted) {
            ++it->second.count;
        }
        route_lookup_size = routes.size();
        route_lookup_data = routes.data();
    }

    void eraseRouteBinding(size_t index)
    {
        if (index >= routes.size()) {
            return;
        }
        routes.erase(routes.begin() + static_cast<std::ptrdiff_t>(index));
        invalidateRouteLookup();
        invalidateRouteIdLookup();
    }

    void clearRouteBindings()
    {
        routes.clear();
        invalidateRouteLookup();
        invalidateRouteIdLookup();
    }

    void invalidateRouteLookup() const
    {
        route_lookup_size = std::numeric_limits<size_t>::max();
        route_lookup_data = nullptr;
    }

    // Assign and return a stable identity that survives vector relocation and
    // index shifts caused by removing another physical route binding.
    uint64_t routeId(size_t index)
    {
        if (index >= routes.size()) {
            return 0;
        }
        if (routes[index].route_id == 0) {
            routes[index].route_id = next_route_id++;
            if (routeIdLookupCurrent()) {
                route_id_lookup[routes[index].route_id] = index;
            } else {
                invalidateRouteIdLookup();
            }
        }
        return routes[index].route_id;
    }

    // Resolve a tile-local stable binding identity in expected constant time.
    size_t findRouteBindingById(uint64_t route_id) const
    {
        if (route_id == 0) {
            return std::numeric_limits<size_t>::max();
        }
        if (route_id_lookup_size != routes.size()
            || route_id_lookup_data != routes.data()) {
            route_id_lookup.clear();
            route_id_lookup.reserve(routes.size());
            for (size_t index = 0; index < routes.size(); ++index) {
                if (routes[index].route_id != 0) {
                    route_id_lookup[routes[index].route_id] = index;
                }
            }
            route_id_lookup_size = routes.size();
            route_id_lookup_data = routes.data();
        }
        auto found = route_id_lookup.find(route_id);
        return found == route_id_lookup.end()
            ? std::numeric_limits<size_t>::max() : found->second;
    }

    void invalidateRouteIdLookup() const
    {
        route_id_lookup_size = std::numeric_limits<size_t>::max();
        route_id_lookup_data = nullptr;
    }

    bool routeCanBePreempted() const
    {
        return !route_protected;
    }

    bool designatorIsVoid(int designator) const
    {
        return void_net
            || std::find(void_designators.begin(), void_designators.end(), designator)
                != void_designators.end();
    }

    bool markDesignatorVoid(int designator)
    {
        if (designatorIsVoid(designator)) {
            return false;
        }
        void_designators.push_back(designator);
        void_net = !designators.empty()
            && std::all_of(designators.begin(), designators.end(),
                [&](int candidate) {
                    return std::find(void_designators.begin(), void_designators.end(), candidate)
                        != void_designators.end();
                });
        return true;
    }

    void clearVoidDesignators()
    {
        void_net = false;
        void_designators.clear();
    }

    std::string makeName(size_t limit = 200)
    {
        return name;
    }
};


}
