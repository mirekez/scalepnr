#pragma once

#include "json/json.h"

#include <cstdint>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

namespace db {

struct PnrDbCoord
{
    int x = -1;
    int y = -1;
};

struct PnrDbEndpoint
{
    std::string inst;
    std::string port;
    uint32_t node = std::numeric_limits<uint32_t>::max();
};

struct PnrDbRouteBranch
{
    uint32_t logical_net_index = std::numeric_limits<uint32_t>::max();
    uint32_t owner_route_index = std::numeric_limits<uint32_t>::max();
    std::string logical_net;
    std::string route_name;
    std::string owner;
    PnrDbEndpoint source;
    PnrDbEndpoint sink;
    std::vector<Json::Value> wires;
};

struct PnrDbRouteNode
{
    uint32_t id = 0;
    PnrDbCoord coord;
    std::string kind;
    int node = -1;
    std::string name;
};

struct PnrDbRouteEdge
{
    uint32_t from = 0;
    uint32_t to = 0;
    Json::Value wire;
};

struct PnrDbRouteTree
{
    std::string id;
    std::string net;
    std::vector<std::string> aliases;
    PnrDbEndpoint source;
    std::vector<PnrDbEndpoint> sinks;
    std::vector<PnrDbRouteBranch> branches;
    std::vector<PnrDbRouteNode> nodes;
    std::vector<PnrDbRouteEdge> edges;
};

inline Json::Value coordToJson(const PnrDbCoord& coord)
{
    Json::Value out(Json::arrayValue);
    out.append(coord.x);
    out.append(coord.y);
    return out;
}

inline PnrDbCoord coordFromJson(const Json::Value& value)
{
    if (!value.isArray() || value.size() < 2) {
        return {};
    }
    return PnrDbCoord{value[0].asInt(), value[1].asInt()};
}

inline Json::Value endpointToJson(const PnrDbEndpoint& endpoint)
{
    Json::Value out(Json::objectValue);
    out["inst"] = endpoint.inst;
    out["port"] = endpoint.port;
    if (endpoint.node != std::numeric_limits<uint32_t>::max()) {
        out["node"] = Json::UInt64(endpoint.node);
    }
    return out;
}

inline PnrDbEndpoint endpointFromJson(const Json::Value& value)
{
    PnrDbEndpoint out;
    out.inst = value.get("inst", "").asString();
    out.port = value.get("port", "").asString();
    if (value.isMember("node")) {
        out.node = value["node"].asUInt();
    }
    return out;
}

inline Json::Value routeBranchToJson(const PnrDbRouteBranch& branch)
{
    Json::Value out(Json::objectValue);
    out["logical_net_index"] = Json::UInt64(branch.logical_net_index);
    out["owner_route_index"] = Json::UInt64(branch.owner_route_index);
    out["logical_net"] = branch.logical_net;
    out["route_name"] = branch.route_name;
    out["owner"] = branch.owner;
    out["source"] = endpointToJson(branch.source);
    out["sink"] = endpointToJson(branch.sink);
    Json::Value wires(Json::arrayValue);
    for (const Json::Value& wire : branch.wires) {
        wires.append(wire);
    }
    out["wires"] = std::move(wires);
    return out;
}

inline PnrDbRouteBranch routeBranchFromJson(const Json::Value& value)
{
    PnrDbRouteBranch out;
    out.logical_net_index = value.get("logical_net_index", Json::UInt64(std::numeric_limits<uint32_t>::max())).asUInt();
    out.owner_route_index = value.get("owner_route_index", Json::UInt64(std::numeric_limits<uint32_t>::max())).asUInt();
    out.logical_net = value.get("logical_net", "").asString();
    out.route_name = value.get("route_name", "").asString();
    out.owner = value.get("owner", "").asString();
    out.source = endpointFromJson(value["source"]);
    out.sink = endpointFromJson(value["sink"]);
    for (const Json::Value& wire : value["wires"]) {
        out.wires.push_back(wire);
    }
    return out;
}

inline Json::Value routeNodeToJson(const PnrDbRouteNode& node)
{
    Json::Value out(Json::objectValue);
    out["id"] = Json::UInt64(node.id);
    out["coord"] = coordToJson(node.coord);
    out["kind"] = node.kind;
    out["node"] = node.node;
    out["name"] = node.name;
    return out;
}

inline PnrDbRouteNode routeNodeFromJson(const Json::Value& value)
{
    PnrDbRouteNode out;
    out.id = value.get("id", 0).asUInt();
    out.coord = coordFromJson(value["coord"]);
    out.kind = value.get("kind", "").asString();
    out.node = value.get("node", -1).asInt();
    out.name = value.get("name", "").asString();
    return out;
}

inline Json::Value routeEdgeToJson(const PnrDbRouteEdge& edge)
{
    Json::Value out(Json::objectValue);
    out["from"] = Json::UInt64(edge.from);
    out["to"] = Json::UInt64(edge.to);
    out["wire"] = edge.wire;
    return out;
}

inline PnrDbRouteEdge routeEdgeFromJson(const Json::Value& value)
{
    PnrDbRouteEdge out;
    out.from = value.get("from", 0).asUInt();
    out.to = value.get("to", 0).asUInt();
    out.wire = value["wire"];
    return out;
}

inline Json::Value routeTreeToJson(const PnrDbRouteTree& tree)
{
    Json::Value out(Json::objectValue);
    out["id"] = tree.id;
    out["net"] = tree.net;
    out["source"] = endpointToJson(tree.source);

    Json::Value aliases(Json::arrayValue);
    for (const std::string& alias : tree.aliases) {
        aliases.append(alias);
    }
    out["aliases"] = std::move(aliases);

    Json::Value sinks(Json::arrayValue);
    for (const PnrDbEndpoint& sink : tree.sinks) {
        sinks.append(endpointToJson(sink));
    }
    out["sinks"] = sinks;

    Json::Value branches(Json::arrayValue);
    for (const PnrDbRouteBranch& branch : tree.branches) {
        branches.append(routeBranchToJson(branch));
    }
    out["branches"] = std::move(branches);

    Json::Value nodes(Json::arrayValue);
    for (const PnrDbRouteNode& node : tree.nodes) {
        nodes.append(routeNodeToJson(node));
    }
    out["nodes"] = nodes;

    Json::Value edges(Json::arrayValue);
    for (const PnrDbRouteEdge& edge : tree.edges) {
        edges.append(routeEdgeToJson(edge));
    }
    out["edges"] = edges;
    return out;
}

inline PnrDbRouteTree routeTreeFromJson(const Json::Value& value)
{
    PnrDbRouteTree out;
    out.id = value.get("id", "").asString();
    out.net = value.get("net", "").asString();
    for (const Json::Value& alias : value["aliases"]) {
        out.aliases.push_back(alias.asString());
    }
    out.source = endpointFromJson(value["source"]);
    for (const Json::Value& sink : value["sinks"]) {
        out.sinks.push_back(endpointFromJson(sink));
    }
    for (const Json::Value& branch : value["branches"]) {
        out.branches.push_back(routeBranchFromJson(branch));
    }
    for (const Json::Value& node : value["nodes"]) {
        out.nodes.push_back(routeNodeFromJson(node));
    }
    for (const Json::Value& edge : value["edges"]) {
        out.edges.push_back(routeEdgeFromJson(edge));
    }
    return out;
}

inline bool routeTreeHasConsistentIds(const PnrDbRouteTree& tree)
{
    std::unordered_set<uint32_t> node_ids;
    for (const PnrDbRouteNode& node : tree.nodes) {
        if (!node_ids.insert(node.id).second) {
            return false;
        }
    }
    for (const PnrDbRouteEdge& edge : tree.edges) {
        if (!node_ids.contains(edge.from) || !node_ids.contains(edge.to)) {
            return false;
        }
    }
    return true;
}

}
