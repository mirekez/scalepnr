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

    const std::string* localResourceName(TilePinNameType type, int local_node) const
    {
        auto it = local_resource_names.find(TilePinNameKey{static_cast<uint8_t>(type), local_node});
        return it == local_resource_names.end() ? nullptr : &it->second;
    }

    const std::string* localPinName(TilePinNameType type, int local_node) const
    {
        auto it = local_pin_names.find(TilePinNameKey{static_cast<uint8_t>(type), local_node});
        return it == local_pin_names.end() ? nullptr : &it->second;
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
