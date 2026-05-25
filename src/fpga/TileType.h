#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "Types.h"
#include "Pin.h"
#include "u256.h"

namespace fpga {

enum TilePinNameType : uint8_t
{
    TILE_PIN_INPUT,
    TILE_PIN_OUTPUT,
};

struct TilePinNameKey
{
    uint8_t type;
    int value;

    bool operator==(const TilePinNameKey& other) const
    {
        return type == other.type && value == other.value;
    }
};

struct TilePinNameKeyHash
{
    std::size_t operator()(const TilePinNameKey& key) const
    {
        return (static_cast<std::size_t>(key.type) << 16) ^ static_cast<std::size_t>(key.value);
    }
};

struct TilePinEndpointNameKey
{
    uint8_t type;
    int resource;
    int local;

    bool operator==(const TilePinEndpointNameKey& other) const
    {
        return type == other.type && resource == other.resource && local == other.local;
    }
};

struct TilePinEndpointNameKeyHash
{
    std::size_t operator()(const TilePinEndpointNameKey& key) const
    {
        return (static_cast<std::size_t>(key.type) << 24)
            ^ (static_cast<std::size_t>(key.resource) << 8)
            ^ static_cast<std::size_t>(key.local);
    }
};

struct TilePinKey
{
    std::string cell_type;
    std::string port;
    int pos = -1;

    bool operator<(const TilePinKey& other) const
    {
        if (cell_type != other.cell_type) return cell_type < other.cell_type;
        if (port != other.port) return port < other.port;
        return pos < other.pos;
    }
};

struct TilePinMap
{
    std::map<TilePinKey, u256> nodes;
    std::map<int, u256> input_nodes;
    std::map<int, u256> output_nodes;
    std::unordered_map<TilePinNameKey, std::string, TilePinNameKeyHash> local_wire_names;
    std::unordered_map<TilePinNameKey, std::string, TilePinNameKeyHash> local_resource_names;
    std::unordered_map<TilePinNameKey, std::string, TilePinNameKeyHash> local_pin_names;
    std::unordered_map<TilePinNameKey, std::string, TilePinNameKeyHash> resource_pin_names;
    std::unordered_map<TilePinEndpointNameKey, std::string, TilePinEndpointNameKeyHash> endpoint_wire_names;
    std::unordered_map<TilePinEndpointNameKey, std::string, TilePinEndpointNameKeyHash> endpoint_resource_names;
    std::unordered_map<TilePinEndpointNameKey, std::string, TilePinEndpointNameKeyHash> endpoint_pin_names;

    u256 getNodes(const std::string& cell_type, const std::string& port, int pos) const
    {
        auto it = nodes.find(TilePinKey{cell_type, port, pos});
        return it == nodes.end() ? u256{} : it->second;
    }

    u256 getInputNodes(int resource_node) const
    {
        auto it = input_nodes.find(resource_node);
        return it == input_nodes.end() ? u256{} : it->second;
    }

    u256 getOutputNodes(int resource_node) const
    {
        auto it = output_nodes.find(resource_node);
        return it == output_nodes.end() ? u256{} : it->second;
    }

    u256 getNodesForPin(TilePinNameType type, const std::string& pin) const
    {
        u256 nodes;
        if (pin.empty()) {
            return nodes;
        }
        const std::map<int, u256>& by_resource = type == TILE_PIN_INPUT ? input_nodes : output_nodes;
        for (const auto& entry : resource_pin_names) {
            if (entry.first.type != static_cast<uint8_t>(type) || entry.second != pin) {
                continue;
            }
            auto it = by_resource.find(entry.first.value);
            if (it != by_resource.end()) {
                nodes |= it->second;
            }
        }
        return nodes;
    }

    int findResourceNode(TilePinNameType type, const std::string& pin, int local_node, int preferred_resource) const
    {
        if (local_node < 0) {
            return -1;
        }
        auto has_endpoint = [&](int resource) {
            if (resource < 0) {
                return false;
            }
            auto resource_it = resource_pin_names.find(TilePinNameKey{static_cast<uint8_t>(type), resource});
            if (resource_it != resource_pin_names.end() && !pin.empty() && resource_it->second != pin) {
                return false;
            }
            return endpoint_wire_names.find(TilePinEndpointNameKey{static_cast<uint8_t>(type), resource, local_node}) != endpoint_wire_names.end()
                || endpoint_resource_names.find(TilePinEndpointNameKey{static_cast<uint8_t>(type), resource, local_node}) != endpoint_resource_names.end();
        };
        if (has_endpoint(preferred_resource)) {
            return preferred_resource;
        }
        const std::map<int, u256>& by_resource = type == TILE_PIN_INPUT ? input_nodes : output_nodes;
        for (const auto& [resource, nodes] : by_resource) {
            if ((nodes & (u256{0,1} << local_node)) == u256{}) {
                continue;
            }
            if (has_endpoint(resource)) {
                return resource;
            }
        }
        return preferred_resource;
    }

    void rememberLocalNames(TilePinNameType type, int local_node, const std::string& local_wire,
                            const std::string& resource_wire, const std::string& pin)
    {
        if (local_node < 0) {
            return;
        }
        TilePinNameKey key{static_cast<uint8_t>(type), local_node};
        if (!local_wire.empty()) {
            local_wire_names.try_emplace(key, local_wire);
        }
        if (!resource_wire.empty()) {
            local_resource_names.try_emplace(key, resource_wire);
        }
        if (!pin.empty()) {
            local_pin_names.try_emplace(key, pin);
        }
    }

    void rememberEndpointNames(TilePinNameType type, int resource_node, int local_node,
                               const std::string& local_wire, const std::string& resource_wire,
                               const std::string& pin)
    {
        if (resource_node < 0 || local_node < 0) {
            return;
        }
        TilePinEndpointNameKey key{static_cast<uint8_t>(type), resource_node, local_node};
        if (!local_wire.empty()) {
            endpoint_wire_names.try_emplace(key, local_wire);
        }
        if (!resource_wire.empty()) {
            endpoint_resource_names.try_emplace(key, resource_wire);
        }
        if (!pin.empty()) {
            endpoint_pin_names.try_emplace(key, pin);
        }
    }

    void rememberResourcePinName(TilePinNameType type, int resource_node, const std::string& pin)
    {
        if (resource_node < 0 || pin.empty()) {
            return;
        }
        resource_pin_names.try_emplace(TilePinNameKey{static_cast<uint8_t>(type), resource_node}, pin);
    }

    const std::string* localWireName(TilePinNameType type, int local_node) const
    {
        auto it = local_wire_names.find(TilePinNameKey{static_cast<uint8_t>(type), local_node});
        return it == local_wire_names.end() ? nullptr : &it->second;
    }

    const std::string* localWireName(TilePinNameType type, int resource_node, int local_node) const
    {
        auto it = endpoint_wire_names.find(TilePinEndpointNameKey{static_cast<uint8_t>(type), resource_node, local_node});
        return it == endpoint_wire_names.end() ? nullptr : &it->second;
    }

    const std::string* localResourceName(TilePinNameType type, int local_node) const
    {
        auto it = local_resource_names.find(TilePinNameKey{static_cast<uint8_t>(type), local_node});
        return it == local_resource_names.end() ? nullptr : &it->second;
    }

    const std::string* localResourceName(TilePinNameType type, int resource_node, int local_node) const
    {
        auto it = endpoint_resource_names.find(TilePinEndpointNameKey{static_cast<uint8_t>(type), resource_node, local_node});
        return it == endpoint_resource_names.end() ? nullptr : &it->second;
    }

    const std::string* localPinName(TilePinNameType type, int local_node) const
    {
        auto it = local_pin_names.find(TilePinNameKey{static_cast<uint8_t>(type), local_node});
        return it == local_pin_names.end() ? nullptr : &it->second;
    }

    const std::string* localPinName(TilePinNameType type, int resource_node, int local_node) const
    {
        auto it = endpoint_pin_names.find(TilePinEndpointNameKey{static_cast<uint8_t>(type), resource_node, local_node});
        return it == endpoint_pin_names.end() ? nullptr : &it->second;
    }

    const std::string* resourcePinName(TilePinNameType type, int resource_node) const
    {
        auto it = resource_pin_names.find(TilePinNameKey{static_cast<uint8_t>(type), resource_node});
        return it == resource_pin_names.end() ? nullptr : &it->second;
    }
};

struct TilePinState
{
    u256 leased_nodes;

    bool lease(int local)
    {
        u256 prev = leased_nodes;
        leased_nodes |= u256{0,1} << local;
        return leased_nodes != prev;
    }
};

struct TileType
{
    // must have
    std::string name;
    size_t num; //?
    int type;

    // optional
    TilePinMap pin_map;
};

}
