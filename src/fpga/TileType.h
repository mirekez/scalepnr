#pragma once

#include <map>
#include <string>
#include <vector>

#include "Types.h"
#include "Pin.h"
#include "u256.h"

namespace fpga {

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
