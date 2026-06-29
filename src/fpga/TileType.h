#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <cstdio>

#include "Types.h"
#include "Element.h"
#include "Pin.h"
#include "NodeMask.h"

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
    // Exact resource/local pair for a tile-pin endpoint annotation.
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
    // Hash endpoint annotation keys without depending on vendor-specific names.
    std::size_t operator()(const TilePinEndpointNameKey& key) const
    {
        return (static_cast<std::size_t>(key.type) << 24)
            ^ (static_cast<std::size_t>(key.resource) << 8)
            ^ static_cast<std::size_t>(key.local);
    }
};

struct TilePinEndpointRouteRef
{
    std::string route_type;
    Coord delta;
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
    std::map<TilePinKey, NodeMask> nodes;
    std::map<int, NodeMask> input_nodes;
    std::map<int, NodeMask> output_nodes;
    std::unordered_map<TilePinNameKey, std::string, TilePinNameKeyHash> local_wire_names;
    std::unordered_map<TilePinNameKey, std::string, TilePinNameKeyHash> local_resource_names;
    std::unordered_map<TilePinNameKey, std::string, TilePinNameKeyHash> local_pin_names;
    std::unordered_map<TilePinNameKey, std::string, TilePinNameKeyHash> resource_pin_names;
    std::unordered_map<TilePinNameKey, std::vector<std::string>, TilePinNameKeyHash> resource_pin_aliases;
    // Endpoint-specific names disambiguate one local node feeding several resource pins.
    std::unordered_map<TilePinEndpointNameKey, std::string, TilePinEndpointNameKeyHash> endpoint_wire_names;
    std::unordered_map<TilePinEndpointNameKey, std::string, TilePinEndpointNameKeyHash> endpoint_resource_names;
    std::unordered_map<TilePinEndpointNameKey, std::string, TilePinEndpointNameKeyHash> endpoint_pin_names;
    std::unordered_map<TilePinEndpointNameKey, std::vector<TilePinEndpointRouteRef>, TilePinEndpointNameKeyHash> endpoint_route_refs;

    NodeMask getNodes(const std::string& cell_type, const std::string& port, int pos) const
    {
        auto it = nodes.find(TilePinKey{cell_type, port, pos});
        return it == nodes.end() ? NodeMask{} : it->second;
    }

    NodeMask getInputNodes(int resource_node) const
    {
        auto it = input_nodes.find(resource_node);
        return it == input_nodes.end() ? NodeMask{} : it->second;
    }

    NodeMask getOutputNodes(int resource_node) const
    {
        auto it = output_nodes.find(resource_node);
        return it == output_nodes.end() ? NodeMask{} : it->second;
    }

    NodeMask getNodesForPin(TilePinNameType type, const std::string& pin, int site_pos = -1,
                        const std::string& route_type = std::string{}, bool strict_route_type = false,
                        const Coord* route_delta = nullptr) const
    {
        // Collect all local nodes that can reach a named resource pin.
        NodeMask nodes{};
        NodeMask filtered_nodes{};
        if (pin.empty()) {
            return nodes;
        }
        const std::map<int, NodeMask>& by_resource = type == TILE_PIN_INPUT ? input_nodes : output_nodes;
        for (int resource : resourceNodesForPin(type, pin, site_pos)) {
            auto it = by_resource.find(resource);
            if (it != by_resource.end()) {
                NodeMask resource_nodes = it->second;
                nodes |= resource_nodes;
                if (!route_type.empty()) {
                    NodeMask filtered{};
                    resource_nodes.for_each_set_bit([&](int local) {
                        TilePinEndpointNameKey endpoint{static_cast<uint8_t>(type), resource, local};
                        auto route_it = endpoint_route_refs.find(endpoint);
                        bool route_ok = (!strict_route_type && route_it == endpoint_route_refs.end())
                            || (route_it != endpoint_route_refs.end()
                            && std::any_of(route_it->second.begin(), route_it->second.end(),
                                [&](const TilePinEndpointRouteRef& ref) {
                                    return ref.route_type == route_type
                                        && (!route_delta || (ref.delta.x == route_delta->x && ref.delta.y == route_delta->y));
                                }));
                        if (route_ok) {
                            filtered |= NodeMask{0,1} << local;
                        }
                        return false;
                    });
                    filtered_nodes |= filtered;
                }
            }
        }
        if (route_type.empty()) {
            return nodes;
        }
        if (strict_route_type) {
            return filtered_nodes;
        }
        return filtered_nodes != NodeMask{} ? filtered_nodes : nodes;
    }

    int findResourceNode(TilePinNameType type, const std::string& pin, int local_node, int preferred_resource, int site_pos = -1) const
    {
        // Resolve the resource endpoint that matches the selected local node.
        if (local_node < 0) {
            return -1;
        }
        auto has_endpoint = [&](int resource) {
            if (resource < 0) {
                return false;
            }
            if (!pin.empty() && !resourcePinMatches(type, resource, pin)) {
                return false;
            }
            return endpoint_wire_names.find(TilePinEndpointNameKey{static_cast<uint8_t>(type), resource, local_node}) != endpoint_wire_names.end()
                || endpoint_resource_names.find(TilePinEndpointNameKey{static_cast<uint8_t>(type), resource, local_node}) != endpoint_resource_names.end();
        };
        auto has_any_endpoint = [&](int resource) {
            // Accept the concrete local endpoint even when logical pin mapping is permuted.
            if (resource < 0) {
                return false;
            }
            return endpoint_wire_names.find(TilePinEndpointNameKey{static_cast<uint8_t>(type), resource, local_node}) != endpoint_wire_names.end()
                || endpoint_resource_names.find(TilePinEndpointNameKey{static_cast<uint8_t>(type), resource, local_node}) != endpoint_resource_names.end();
        };
        if (has_endpoint(preferred_resource)) {
            return preferred_resource;
        }
        const std::map<int, NodeMask>& by_resource = type == TILE_PIN_INPUT ? input_nodes : output_nodes;
        int local_endpoint = -1;
        for (const auto& [resource, nodes] : by_resource) {
            if ((nodes & (NodeMask{0,1} << local_node)) == NodeMask{}) {
                continue;
            }
            if (site_pos >= 0 && resource / 256 != site_pos) {
                continue;
            }
            if (local_endpoint < 0 && has_any_endpoint(resource)) {
                local_endpoint = resource;
            }
            if (has_endpoint(resource)) {
                return resource;
            }
        }
        return local_endpoint >= 0 ? local_endpoint : preferred_resource;
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
        // Store the exact local-to-resource names loaded from tile-type data.
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

    void rememberEndpointRouteRef(TilePinNameType type, int resource_node, int local_node,
                                  const std::string& route_type, Coord delta = Coord{0,0})
    {
        // Preserve which adjacent route tile type and relative coordinate produced this endpoint-local mapping.
        if (resource_node < 0 || local_node < 0 || route_type.empty()) {
            return;
        }
        TilePinEndpointNameKey key{static_cast<uint8_t>(type), resource_node, local_node};
        auto& route_refs = endpoint_route_refs[key];
        auto same = [&](const TilePinEndpointRouteRef& ref) {
            return ref.route_type == route_type && ref.delta.x == delta.x && ref.delta.y == delta.y;
        };
        if (std::find_if(route_refs.begin(), route_refs.end(), same) == route_refs.end()) {
            route_refs.push_back(TilePinEndpointRouteRef{route_type, delta});
        }
    }

    void rememberResourcePinName(TilePinNameType type, int resource_node, const std::string& pin)
    {
        if (resource_node < 0 || pin.empty()) {
            return;
        }
        TilePinNameKey key{static_cast<uint8_t>(type), resource_node};
        resource_pin_names.try_emplace(key, pin);
        auto& aliases = resource_pin_aliases[key];
        if (std::find(aliases.begin(), aliases.end(), pin) == aliases.end()) {
            aliases.push_back(pin);
        }
    }

    bool resourcePinMatches(TilePinNameType type, int resource_node, const std::string& pin) const
    {
        TilePinNameKey key{static_cast<uint8_t>(type), resource_node};
        auto aliases = resource_pin_aliases.find(key);
        if (aliases != resource_pin_aliases.end()
            && std::find(aliases->second.begin(), aliases->second.end(), pin) != aliases->second.end()) {
            return true;
        }
        auto primary = resource_pin_names.find(key);
        return primary != resource_pin_names.end() && primary->second == pin;
    }

    std::vector<int> resourceNodesForPin(TilePinNameType type, const std::string& pin, int site_pos = -1) const
    {
        std::vector<int> resources;
        for (const auto& entry : resource_pin_aliases) {
            if (entry.first.type != static_cast<uint8_t>(type)) {
                continue;
            }
            if (site_pos >= 0 && entry.first.value / 256 != site_pos) {
                continue;
            }
            if (std::find(entry.second.begin(), entry.second.end(), pin) != entry.second.end()) {
                resources.push_back(entry.first.value);
            }
        }
        if (!resources.empty()) {
            return resources;
        }
        for (const auto& entry : resource_pin_names) {
            if (entry.first.type != static_cast<uint8_t>(type) || entry.second != pin) {
                continue;
            }
            if (site_pos >= 0 && entry.first.value / 256 != site_pos) {
                continue;
            }
            resources.push_back(entry.first.value);
        }
        return resources;
    }

    const std::string* localWireName(TilePinNameType type, int local_node) const
    {
        auto it = local_wire_names.find(TilePinNameKey{static_cast<uint8_t>(type), local_node});
        return it == local_wire_names.end() ? nullptr : &it->second;
    }

    const std::string* localWireName(TilePinNameType type, int resource_node, int local_node) const
    {
        // Prefer exact endpoint wire names over ambiguous local-only names.
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
        // Look up the resource-side wire for a selected endpoint pair.
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
        // Return the resource pin name associated with an exact endpoint pair.
        auto it = endpoint_pin_names.find(TilePinEndpointNameKey{static_cast<uint8_t>(type), resource_node, local_node});
        return it == endpoint_pin_names.end() ? nullptr : &it->second;
    }

    const std::string* resourcePinName(TilePinNameType type, int resource_node) const
    {
        auto it = resource_pin_names.find(TilePinNameKey{static_cast<uint8_t>(type), resource_node});
        return it == resource_pin_names.end() ? nullptr : &it->second;
    }
};

struct BelModel
{
    // Abstract BEL bucket for resources that share one site placement slot.
    std::string type;
    int pos = -1;
    std::vector<Pin> pins;
};

struct SiteModel
{
    // Database-loaded site description used to map placement to resource pins.
    std::string name;
    std::string type;
    int pos = -1;
    std::vector<Pin> pins;
    std::vector<BelModel> bels;

    const Pin* pinByPort(const std::string& port) const
    {
        // Find a site pin by abstract resource-port name.
        for (const Pin& pin : pins) {
            if (pin.port == port) {
                return &pin;
            }
        }
        return nullptr;
    }
};

struct TilePinState
{
    NodeMask leased_nodes{};

    bool lease(int local)
    {
        NodeMask prev = leased_nodes;
        leased_nodes |= NodeMask{0,1} << local;
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
    std::vector<SiteModel> sites;
    std::vector<Element> elements;

    void rebuildElementsFromSites();

    int sitePosForPlacedPos(int placed_pos) const
    {
        // Decode abstract placement position into a database site position.
        if (sites.empty()) {
            return -1;
        }
        int site_index = placed_pos >= 0 ? placed_pos / 128 : 0;
        if (elements.empty() && placed_pos >= 0) {
            site_index = placed_pos;
        }
        if (site_index < 0 || site_index >= static_cast<int>(sites.size())) {
            site_index = 0;
        }
        return sites[site_index].pos;
    }

    const SiteModel* siteForPlacedPos(int placed_pos) const
    {
        // Return the site model selected by an abstract placement position.
        if (sites.empty()) {
            return nullptr;
        }
        int site_index = placed_pos >= 0 ? placed_pos / 128 : 0;
        if (elements.empty() && placed_pos >= 0) {
            site_index = placed_pos;
        }
        if (site_index < 0 || site_index >= static_cast<int>(sites.size())) {
            return nullptr;
        }
        return &sites[site_index];
    }
};

}
