#include "RouteClocks.h"

#include "Clocks.h"
#include "Device.h"
#include "RouteSearch.h"
#include "Tech.h"
#include "Tile.h"
#include "Wire.h"

#include <algorithm>
#include <cctype>
#include <deque>
#include <format>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

using fpga::CBNodeNameType;
struct RouteNode
{
    fpga::Tile* tile = nullptr;
    CBNodeNameType type = fpga::CB_NODE_LOCAL;
    int value = -1;
};

struct RouteNodeKey
{
    int x = -1;
    int y = -1;
    uint8_t type = 0;
    uint16_t value = 0;

    bool operator==(const RouteNodeKey&) const = default;
};

struct RouteNodeKeyHash
{
    size_t operator()(const RouteNodeKey& key) const
    {
        size_t value = static_cast<size_t>(static_cast<uint32_t>(key.x));
        value = value * 1315423911u + static_cast<uint32_t>(key.y);
        value = value * 1315423911u + key.type;
        return value * 1315423911u + key.value;
    }
};

RouteNodeKey nodeKey(const RouteNode& node)
{
    return RouteNodeKey{node.tile ? node.tile->coord.x : -1,
                        node.tile ? node.tile->coord.y : -1,
                        static_cast<uint8_t>(node.type),
                        static_cast<uint16_t>(std::max(node.value, 0))};
}

std::string describeNode(const RouteNode& node)
{
    std::string name = "#" + std::to_string(node.value);
    if (node.tile && node.tile->cb_type) {
        if (const std::string* loaded = node.tile->cb_type->nodeName(node.type, node.value)) {
            name = *loaded;
        }
    }
    return std::format("{}@({},{}):{}:{}#{}", node.tile && node.tile->cb_type
        ? node.tile->cb_type->name : std::string{"?"}, node.tile ? node.tile->coord.x : -1,
        node.tile ? node.tile->coord.y : -1, static_cast<int>(node.type), name, node.value);
}

struct Endpoint
{
    fpga::Tile* route_tile = nullptr;
    fpga::Tile* resource_tile = nullptr;
    int local = -1;
    int resource_node = -1;
    std::string physical_pin;
};

struct ParentEdge
{
    RouteNode parent;
    bool valid = false;
    int tile_visits_root = -1;
};

struct ClockTree
{
    RouteNode root;
    Endpoint root_endpoint;
    pnr::CombinatorialTileVisits tile_visits;
    std::unordered_map<RouteNodeKey, ParentEdge, RouteNodeKeyHash> parents;
    std::unordered_set<RouteNodeKey, RouteNodeKeyHash> nodes;
};

std::string portStem(std::string name)
{
    while (!name.empty() && std::isdigit(static_cast<unsigned char>(name.back()))) {
        name.pop_back();
    }
    return name;
}

bool compatiblePort(const std::string& logical, const std::string& physical)
{
    return logical == physical || (!logical.empty() && portStem(logical) == portStem(physical));
}

bool compatibleSiteType(const rtl::Inst& inst, const fpga::SiteModel& site)
{
    if (!inst.cell_ref.peer || inst.cell_ref.peer->type.empty() || site.type.empty()) {
        return false;
    }
    const std::string& cell_type = inst.cell_ref.peer->type;
    return cell_type == site.type || site.type.starts_with(cell_type)
        || cell_type.starts_with(site.type);
}

bool pinDirectionMatches(const fpga::Pin& pin, fpga::TilePinNameType direction)
{
    return pin.direction == fpga::Pin::PIN_INOUT
        || (direction == fpga::TILE_PIN_INPUT && pin.direction == fpga::Pin::PIN_INPUT)
        || (direction == fpga::TILE_PIN_OUTPUT && pin.direction == fpga::Pin::PIN_OUTPUT);
}

rtl::Net* findNetByDesignator(rtl::Inst& inst, int designator)
{
    if (!inst.cell_ref.peer || !inst.cell_ref->module_ref.peer) {
        return nullptr;
    }
    rtl::Module* parent = inst.cell_ref->module_ref->parent_ref.peer;
    if (!parent) {
        return nullptr;
    }
    for (auto& net : parent->nets) {
        if (std::find(net.designators.begin(), net.designators.end(), designator)
            != net.designators.end()) {
            return &net;
        }
    }
    return nullptr;
}

void collectLeafInsts(rtl::Inst& inst, std::vector<rtl::Inst*>& out)
{
    if (inst.cell_ref.peer && inst.cell_ref->module_ref.peer
        && inst.cell_ref->module_ref->is_blackbox) {
        out.push_back(&inst);
        return;
    }
    for (auto& child : inst.insts) {
        collectLeafInsts(child, out);
    }
}

rtl::Inst* instFromTileRef(RefBase<Referable<fpga::Tile>>* tile_ref)
{
    if (!tile_ref) {
        return nullptr;
    }
    Ref<fpga::Tile>* ref = Ref<fpga::Tile>::fromBase(tile_ref);
    return reinterpret_cast<rtl::Inst*>(reinterpret_cast<char*>(ref) - offsetof(rtl::Inst, tile));
}

bool siteOccupied(fpga::Tile& tile, int site_index)
{
    for (RefBase<Referable<fpga::Tile>>* peer :
         static_cast<Referable<fpga::Tile>*>(&tile)->getPeers()) {
        rtl::Inst* inst = instFromTileRef(peer);
        if (inst && inst->pos == site_index) {
            return true;
        }
    }
    return false;
}

std::vector<const fpga::Pin*> matchingPins(const fpga::SiteModel& site,
                                            const std::string& logical_port,
                                            fpga::TilePinNameType direction)
{
    std::vector<const fpga::Pin*> exact;
    std::vector<const fpga::Pin*> compatible;
    for (const fpga::Pin& pin : site.pins) {
        if (!pinDirectionMatches(pin, direction)) {
            continue;
        }
        if (pin.port == logical_port) {
            exact.push_back(&pin);
        }
        else if (compatiblePort(logical_port, pin.port)) {
            compatible.push_back(&pin);
        }
    }
    exact.insert(exact.end(), compatible.begin(), compatible.end());
    return exact;
}

std::vector<Endpoint> endpointNodes(fpga::Device& device, rtl::Inst& inst,
                                    const std::string& logical_port,
                                    fpga::TilePinNameType direction)
{
    std::vector<Endpoint> result;
    if (!inst.tile.peer || !inst.cell_ref.peer) {
        return result;
    }
    fpga::Tile* resource = inst.tile.peer;
    auto append = [&](fpga::Tile* route, int local, int resource_node,
                      const std::string& physical_pin) {
        if (!route || !route->cb_type) {
            return;
        }
        if (local < 0 || local >= CB_MAX_NODES) {
            return;
        }
        auto duplicate = std::find_if(result.begin(), result.end(), [&](const Endpoint& endpoint) {
            return endpoint.route_tile == route && endpoint.local == local
                && endpoint.resource_node == resource_node;
        });
        if (duplicate == result.end()) {
            result.push_back(Endpoint{route, resource, local, resource_node, physical_pin});
        }
    };

    const fpga::SiteModel* site = resource->tile_type
        ? resource->tile_type->siteForPlacedPos(inst.pos) : nullptr;
    auto appendRoute = [&](fpga::Tile* route) {
        if (!route || !route->cb_type || !resource->tile_type) {
            return;
        }
        fpga::Coord route_delta{route->coord.x - resource->coord.x,
                                route->coord.y - resource->coord.y};
        // Direct port matching is valid only when the placed cell is the modeled site resource.
        std::vector<const fpga::Pin*> pins = site && compatibleSiteType(inst, *site)
            ? matchingPins(*site, logical_port, direction) : std::vector<const fpga::Pin*>{};
        auto appendNodes = [&](NodeMask nodes, const std::string& physical_pin) {
            nodes.for_each_set_bit([&](int local) {
                int resource_node = resource->tile_type->pin_map.findResourceNode(
                    direction, physical_pin, local, -1, site ? site->pos : inst.pos);
                append(route, local, resource_node, physical_pin);
                return false;
            });
        };
        for (const fpga::Pin* pin : pins) {
            NodeMask nodes{};
            if (route->tile_type && !route->tile_type->name.empty()) {
                nodes = resource->tile_type->pin_map.getNodesForPin(
                    direction, pin->port, site->pos, route->tile_type->name, true, &route_delta);
            }
            if (nodes == NodeMask{}) {
                nodes = resource->tile_type->pin_map.getNodesForPin(
                    direction, pin->port, site->pos, route->cb_type->name, true, &route_delta);
            }
            if (nodes == NodeMask{} && route->coord.x == resource->coord.x
                && route->coord.y == resource->coord.y) {
                nodes = resource->tile_type->pin_map.getNodesForPin(
                    direction, pin->port, site->pos);
            }
            appendNodes(nodes, pin->port);
        }
        NodeMask nodes{};
        if (route->tile_type && !route->tile_type->name.empty()) {
            nodes = resource->getPinNodesForRouteType(inst.cell_ref->type, logical_port,
                inst.pos, direction, route->tile_type->name, route_delta);
        }
        if (nodes == NodeMask{}) {
            nodes = resource->getPinNodesForRouteType(inst.cell_ref->type, logical_port,
                inst.pos, direction, route->cb_type->name, route_delta);
        }
        appendNodes(nodes, logical_port);
    };

    appendRoute(device.routeTile(*resource));
    if (result.empty()) {
        constexpr int endpoint_radius = 6;
        for (int distance = 1; distance <= endpoint_radius && result.empty(); ++distance) {
            for (int dy = -distance; dy <= distance; ++dy) {
                for (int dx = -distance; dx <= distance; ++dx) {
                    if (std::abs(dx) + std::abs(dy) != distance) {
                        continue;
                    }
                    fpga::Tile* physical = device.getTile(resource->coord.x + dx,
                                                          resource->coord.y + dy);
                    if (physical) {
                        appendRoute(device.routeTile(*physical));
                    }
                }
            }
        }
    }

    if (result.empty()) {
        NodeMask fallback = direction == fpga::TILE_PIN_OUTPUT
            ? resource->getOutputPinNodes(inst.cell_ref->type, logical_port, inst.pos)
            : resource->getPinNodes(inst.cell_ref->type, logical_port, inst.pos);
        fpga::Tile* route = device.routeTile(*resource);
        fallback.for_each_set_bit([&](int local) {
            append(route, local, resource->getResourceNodeNum(inst.cell_ref->type, logical_port,
                inst.pos, direction, local), logical_port);
            return false;
        });
    }
    return result;
}

bool nodeBusy(const RouteNode& node)
{
    if (!node.tile || node.value < 0) {
        return true;
    }
    NodeMask bit = NodeMask{0, 1} << node.value;
    switch (node.type) {
    case fpga::CB_NODE_LOCAL: return (node.tile->cb.local.local & bit) != NodeMask{};
    case fpga::CB_NODE_JOINT: return (node.tile->cb.joint.jump & bit) != NodeMask{};
    case fpga::CB_NODE_SRC: return (node.tile->cb.src.jump & bit) != NodeMask{};
    case fpga::CB_NODE_DST: return (node.tile->cb.dst.jump & bit) != NodeMask{};
    default: return true;
    }
}

void leaseNode(const RouteNode& node)
{
    NodeMask bit = NodeMask{0, 1} << node.value;
    switch (node.type) {
    case fpga::CB_NODE_LOCAL: node.tile->cb.local.local |= bit; break;
    case fpga::CB_NODE_JOINT: node.tile->cb.joint.jump |= bit; break;
    case fpga::CB_NODE_SRC: node.tile->cb.src.jump |= bit; break;
    case fpga::CB_NODE_DST: node.tile->cb.dst.jump |= bit; break;
    default: break;
    }
}

void releaseNode(const RouteNode& node)
{
    NodeMask keep = ~(NodeMask{0, 1} << node.value);
    switch (node.type) {
    case fpga::CB_NODE_LOCAL: node.tile->cb.local.local &= keep; break;
    case fpga::CB_NODE_JOINT: node.tile->cb.joint.jump &= keep; break;
    case fpga::CB_NODE_SRC: node.tile->cb.src.jump &= keep; break;
    case fpga::CB_NODE_DST: node.tile->cb.dst.jump &= keep; break;
    default: break;
    }
}

template<typename State, typename Append>
void appendMask(const State& state, Append append)
{
    state.for_each_set_bit([&](int value) {
        append(value);
        return false;
    });
}

std::vector<RouteNode> outgoing(fpga::Device& device, const RouteNode& node)
{
    std::vector<RouteNode> result;
    if (!node.tile || !node.tile->cb_type || node.value < 0) {
        return result;
    }
    const fpga::CBType& type = *node.tile->cb_type;
    auto same = [&](CBNodeNameType kind, int value) {
        result.push_back(RouteNode{node.tile, kind, value});
    };
    switch (node.type) {
    case fpga::CB_NODE_LOCAL:
        appendMask(type.local_local[node.value].local, [&](int value) { same(fpga::CB_NODE_LOCAL, value); });
        appendMask(type.local_joint[node.value].joint, [&](int value) { same(fpga::CB_NODE_JOINT, value); });
        appendMask(type.local_src[node.value].jump, [&](int value) { same(fpga::CB_NODE_SRC, value); });
        for (const fpga::TileLocalTarget& target : device.resolveLocalTargets(*node.tile, node.value)) {
            result.push_back(RouteNode{target.tile, target.node_type, target.node});
        }
        break;
    case fpga::CB_NODE_DST:
        appendMask(type.dst_local[node.value].local, [&](int value) { same(fpga::CB_NODE_LOCAL, value); });
        appendMask(type.dst_joint[node.value].joint, [&](int value) { same(fpga::CB_NODE_JOINT, value); });
        appendMask(type.dst_src[node.value].jump, [&](int value) { same(fpga::CB_NODE_SRC, value); });
        break;
    case fpga::CB_NODE_JOINT:
        appendMask(type.joint_local[node.value].local, [&](int value) { same(fpga::CB_NODE_LOCAL, value); });
        appendMask(type.joint_joint[node.value].joint, [&](int value) { same(fpga::CB_NODE_JOINT, value); });
        appendMask(type.joint_src[node.value].jump, [&](int value) { same(fpga::CB_NODE_SRC, value); });
        break;
    case fpga::CB_NODE_SRC:
        appendMask(type.src_joint[node.value].joint, [&](int value) { same(fpga::CB_NODE_JOINT, value); });
        for (const fpga::TileJumpTarget& target : device.resolveJumpTargets(*node.tile, node.value)) {
            if (target.tile && target.dst_node >= 0) {
                result.push_back(RouteNode{target.tile, fpga::CB_NODE_DST, target.dst_node});
            }
        }
        break;
    default:
        break;
    }
    return result;
}

std::optional<RouteNode> extendTree(fpga::Device& device, ClockTree& tree,
                                    const std::vector<Endpoint>& targets,
                                    size_t& visited_count,
                                    size_t& tile_visit_rejects)
{
    std::unordered_set<RouteNodeKey, RouteNodeKeyHash> target_keys;
    for (const Endpoint& target : targets) {
        target_keys.insert(nodeKey(RouteNode{target.route_tile, fpga::CB_NODE_LOCAL, target.local}));
    }
    std::deque<RouteNode> queue;
    std::unordered_map<RouteNodeKey, ParentEdge, RouteNodeKeyHash> search_parent;
    for (const RouteNodeKey& key : tree.nodes) {
        auto parent = tree.parents.find(key);
        RouteNode node;
        if (key == nodeKey(tree.root)) {
            node = tree.root;
        }
        else if (parent != tree.parents.end()) {
            node = RouteNode{device.getTile(key.x, key.y), static_cast<CBNodeNameType>(key.type), key.value};
        }
        if (node.tile) {
            queue.push_back(node);
            int visit_root = parent != tree.parents.end()
                ? parent->second.tile_visits_root : -1;
            search_parent.emplace(key, ParentEdge{{}, false, visit_root});
        }
    }

    while (!queue.empty()) {
        RouteNode current = queue.front();
        queue.pop_front();
        ++visited_count;
        RouteNodeKey current_key = nodeKey(current);
        if (target_keys.contains(current_key)) {
            RouteNode cursor = current;
            std::vector<RouteNode> added;
            while (!tree.nodes.contains(nodeKey(cursor))) {
                added.push_back(cursor);
                const ParentEdge& edge = search_parent.at(nodeKey(cursor));
                if (!edge.valid) {
                    return std::nullopt;
                }
                cursor = edge.parent;
            }
            std::reverse(added.begin(), added.end());
            for (const RouteNode& child : added) {
                tree.parents[nodeKey(child)] = search_parent.at(nodeKey(child));
                tree.nodes.insert(nodeKey(child));
                leaseNode(child);
            }
            return current;
        }
        for (const RouteNode& next : outgoing(device, current)) {
            RouteNodeKey key = nodeKey(next);
            if (search_parent.contains(key)) {
                continue;
            }
            if (nodeBusy(next) && !tree.nodes.contains(key)) {
                continue;
            }
            int visit_root = search_parent.at(current_key).tile_visits_root;
            if (next.tile != current.tile &&
                !tree.tile_visits.append(visit_root, next.tile, visit_root)) {
                ++tile_visit_rejects;
                continue;
            }
            search_parent.emplace(key, ParentEdge{current, true, visit_root});
            queue.push_back(next);
        }
    }
    static bool reported_graph = false;
    if (!reported_graph) {
        reported_graph = true;
        std::map<std::string, size_t> by_type;
        std::vector<std::string> terminals;
        for (const auto& [key, parent] : search_parent) {
            (void)parent;
            RouteNode node{device.getTile(key.x, key.y), static_cast<CBNodeNameType>(key.type), key.value};
            if (!node.tile || !node.tile->cb_type) {
                continue;
            }
            ++by_type[node.tile->cb_type->name];
            if (outgoing(device, node).empty() && terminals.size() < 32) {
                terminals.push_back(describeNode(node));
            }
        }
        std::string types;
        for (const auto& [name, count] : by_type) {
            if (!types.empty()) types += ", ";
            types += name + "=" + std::to_string(count);
        }
        std::string terminal_text;
        for (const std::string& terminal : terminals) {
            if (!terminal_text.empty()) terminal_text += ", ";
            terminal_text += terminal;
        }
        PNR_LOG("CLKS", "failed clock graph reachable=[{}] terminals=[{}]", types, terminal_text);
        std::string target_predecessors;
        auto append_predecessor = [&](const RouteNode& predecessor) {
            if (!target_predecessors.empty()) target_predecessors += ", ";
            target_predecessors += describeNode(predecessor);
            target_predecessors += search_parent.contains(nodeKey(predecessor))
                ? "(reached)" : "(unreached)";
        };
        for (const Endpoint& target : targets) {
            if (!target.route_tile || !target.route_tile->cb_type) continue;
            const fpga::CBType& target_type = *target.route_tile->cb_type;
            NodeMask target_bit = NodeMask{0,1} << target.local;
            for (const auto& [local, state] : target_type.local_local.values) {
                if ((state.local & target_bit) != NodeMask{}) {
                    append_predecessor(RouteNode{target.route_tile, fpga::CB_NODE_LOCAL, local});
                }
            }
            for (const auto& [dst, state] : target_type.dst_local.values) {
                if ((state.local & target_bit) != NodeMask{}) {
                    append_predecessor(RouteNode{target.route_tile, fpga::CB_NODE_DST, dst});
                }
            }
            for (const auto& [joint, state] : target_type.joint_local.values) {
                if ((state.local & target_bit) != NodeMask{}) {
                    append_predecessor(RouteNode{target.route_tile, fpga::CB_NODE_JOINT, joint});
                }
            }
            for (auto& tile_ref : device.tile_grid) {
                fpga::Tile& tile = tile_ref;
                if (!tile.cb_type) continue;
                for (const auto& [local, entries] : tile.cb_type->local_by_local.values) {
                    for (const fpga::CBType::ResolvedLocal& entry : entries) {
                        if (tile.coord.x + entry.delta.x != target.route_tile->coord.x
                            || tile.coord.y + entry.delta.y != target.route_tile->coord.y
                            || (target.route_tile->cb_type->base_type_id != CB_INVALID_TYPE_ID
                                    ? target.route_tile->cb_type->base_type_id
                                    : target.route_tile->cb_type->type_id)
                                != entry.target_cb_type_id
                            || entry.target_node_type != fpga::CB_NODE_LOCAL
                            || (entry.target_nodes & target_bit) == NodeMask{}) {
                            continue;
                        }
                        append_predecessor(RouteNode{&tile, fpga::CB_NODE_LOCAL, local});
                    }
                }
            }
            std::string nearby_reached;
            for (const auto& [key, parent] : search_parent) {
                (void)parent;
                if (key.type != fpga::CB_NODE_LOCAL
                    || std::abs(key.x - target.route_tile->coord.x) > 1
                    || std::abs(key.y - target.route_tile->coord.y) > 20) {
                    continue;
                }
                RouteNode reached{device.getTile(key.x, key.y), fpga::CB_NODE_LOCAL, key.value};
                if (!nearby_reached.empty()) nearby_reached += ", ";
                nearby_reached += describeNode(reached);
                if (nearby_reached.size() > 8000) break;
            }
            PNR_LOG("CLKS", "failed clock nearby reached locals=[{}]", nearby_reached);
            auto local_predecessors = [&](const RouteNode& wanted) {
                std::vector<RouteNode> found;
                if (!wanted.tile || !wanted.tile->cb_type || wanted.type != fpga::CB_NODE_LOCAL) {
                    return found;
                }
                NodeMask wanted_bit = NodeMask{0,1} << wanted.value;
                for (const auto& [local, state] : wanted.tile->cb_type->local_local.values) {
                    if ((state.local & wanted_bit) != NodeMask{}) {
                        found.push_back(RouteNode{wanted.tile, fpga::CB_NODE_LOCAL, local});
                    }
                }
                uint16_t wanted_type = wanted.tile->cb_type->base_type_id != CB_INVALID_TYPE_ID
                    ? wanted.tile->cb_type->base_type_id : wanted.tile->cb_type->type_id;
                std::string mismatched;
                for (auto& tile_ref : device.tile_grid) {
                    fpga::Tile& tile = tile_ref;
                    if (!tile.cb_type) continue;
                    for (const auto& [local, entries] : tile.cb_type->local_by_local.values) {
                        for (const fpga::CBType::ResolvedLocal& entry : entries) {
                            bool same_target = tile.coord.x + entry.delta.x == wanted.tile->coord.x
                                && tile.coord.y + entry.delta.y == wanted.tile->coord.y
                                && entry.target_node_type == fpga::CB_NODE_LOCAL
                                && (entry.target_nodes & wanted_bit) != NodeMask{};
                            if (same_target && entry.target_cb_type_id == wanted_type) {
                                found.push_back(RouteNode{&tile, fpga::CB_NODE_LOCAL, local});
                            }
                            else if (same_target && mismatched.size() < 1000) {
                                if (!mismatched.empty()) mismatched += ", ";
                                mismatched += describeNode(RouteNode{&tile, fpga::CB_NODE_LOCAL, local});
                                mismatched += std::format("(entry_type={} wanted_type={})",
                                    entry.target_cb_type_id, wanted_type);
                            }
                        }
                    }
                }
                if (found.empty() && !mismatched.empty()) {
                    PNR_LOG("CLKS", "failed clock local predecessor type mismatches wanted={} entries=[{}]",
                        describeNode(wanted), mismatched);
                }
                return found;
            };
            std::vector<RouteNode> frontier{
                RouteNode{target.route_tile, fpga::CB_NODE_LOCAL, target.local}};
            for (int depth = 1; depth <= 3; ++depth) {
                std::vector<RouteNode> next_frontier;
                std::string level;
                for (const RouteNode& wanted : frontier) {
                    for (const RouteNode& predecessor : local_predecessors(wanted)) {
                        if (!level.empty()) level += ", ";
                        level += describeNode(predecessor);
                        level += search_parent.contains(nodeKey(predecessor))
                            ? "(reached)" : "(unreached)";
                        next_frontier.push_back(predecessor);
                    }
                }
                PNR_LOG("CLKS", "failed clock local predecessor depth={} nodes=[{}]", depth, level);
                frontier = std::move(next_frontier);
            }
        }
        PNR_LOG("CLKS", "failed clock target predecessors=[{}]", target_predecessors);
    }
    return std::nullopt;
}

std::vector<RouteNode> treePath(const ClockTree& tree, RouteNode sink)
{
    std::vector<RouteNode> path{sink};
    while (!(nodeKey(path.back()) == nodeKey(tree.root))) {
        auto it = tree.parents.find(nodeKey(path.back()));
        if (it == tree.parents.end() || !it->second.valid) {
            return {};
        }
        path.push_back(it->second.parent);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

fpga::Wire endpointWire(const Endpoint& endpoint, rtl::Inst& inst,
                        const std::string& port, fpga::TilePinNameType direction,
                        const std::string& net_name)
{
    fpga::Wire wire;
    wire.type = fpga::Wire::WIRE_TILE_PIN;
    wire.from = endpoint.route_tile->coord;
    wire.to = endpoint.route_tile->coord;
    wire.resource = endpoint.resource_tile->coord;
    wire.local = endpoint.local;
    wire.resource_node = endpoint.resource_node;
    wire.pin_dir = direction;
    wire.pos = inst.pos;
    wire.cell_type = inst.cell_ref.peer ? inst.cell_ref->type : std::string{};
    wire.port = endpoint.physical_pin.empty() ? port : endpoint.physical_pin;
    wire.net_name = net_name;
    return wire;
}

std::vector<fpga::Wire> materializePath(const ClockTree& tree,
                                        const Endpoint& sink_endpoint,
                                        rtl::Inst& source, const std::string& source_port,
                                        rtl::Inst& sink, const std::string& sink_port,
                                        const std::string& net_name)
{
    RouteNode sink_node{sink_endpoint.route_tile, fpga::CB_NODE_LOCAL, sink_endpoint.local};
    std::vector<RouteNode> path = treePath(tree, sink_node);
    if (path.empty()) {
        return {};
    }
    std::vector<fpga::Wire> route;
    route.push_back(endpointWire(tree.root_endpoint, source, source_port,
                                 fpga::TILE_PIN_OUTPUT, net_name));
    for (size_t index = 1; index < path.size(); ++index) {
        fpga::Wire edge;
        edge.type = fpga::Wire::WIRE_ROUTE_EDGE;
        edge.from = path[index - 1].tile->coord;
        edge.to = path[index].tile->coord;
        edge.from_node_type = path[index - 1].type;
        edge.from_node = path[index - 1].value;
        edge.to_node_type = path[index].type;
        edge.to_node = path[index].value;
        edge.net_name = net_name;
        route.push_back(std::move(edge));
    }
    route.push_back(endpointWire(sink_endpoint, sink, sink_port,
                                 fpga::TILE_PIN_INPUT, net_name));
    return route;
}

const Endpoint* selectedEndpoint(const std::vector<Endpoint>& endpoints, const RouteNode& node)
{
    auto it = std::find_if(endpoints.begin(), endpoints.end(), [&](const Endpoint& endpoint) {
        return endpoint.route_tile == node.tile && endpoint.local == node.value;
    });
    return it == endpoints.end() ? nullptr : &*it;
}

struct ClockTask
{
    rtl::Inst* from = nullptr;
    rtl::Inst* to = nullptr;
    rtl::Net* net = nullptr;
    std::string from_port;
    std::string to_port;
    std::string net_name;
};

bool placeClockBuffer(fpga::Device& device, rtl::Inst& buffer,
                      const std::vector<ClockTask>& fanout,
                      pnr::RouteClocks::Stats& stats)
{
    if (buffer.tile.peer) {
        return true;
    }
    rtl::Conn* input = nullptr;
    rtl::Conn* output = nullptr;
    for (rtl::Conn& conn : buffer.conns) {
        if (!conn.port_ref.peer) {
            continue;
        }
        if (conn.port_ref->type == rtl::Port::PORT_IN && !input) input = &conn;
        if (conn.port_ref->type == rtl::Port::PORT_OUT && !output) output = &conn;
    }
    if (!input || !output) {
        return false;
    }
    rtl::Conn* driver = input->follow();
    fpga::Coord preferred = driver && driver->inst_ref.peer && driver->inst_ref->tile.peer
        ? driver->inst_ref->tile->coord : fpga::Coord{device.size_width / 2, device.size_height / 2};

    fpga::Tile* best_tile = nullptr;
    int best_site = -1;
    int best_score = std::numeric_limits<int>::max();
    size_t sites_seen = 0;
    size_t compatible_sites = 0;
    size_t pin_sites = 0;
    size_t endpoint_sites = 0;
    std::vector<std::string> sampled_site_types;
    size_t related_tile_reports = 0;
    for (auto& tile_ref : device.tile_grid) {
        fpga::Tile& tile = tile_ref;
        if (!tile.tile_type || !device.routeTile(tile)) {
            continue;
        }
        if (buffer.cell_ref.peer && related_tile_reports < 8
            && tile.tile_type->name.find(buffer.cell_ref.peer->type) != std::string::npos) {
            std::string site_list;
            for (const fpga::SiteModel& site : tile.tile_type->sites) {
                if (!site_list.empty()) site_list += ",";
                site_list += site.type;
            }
            PNR_LOG("CLKS", "clock placement related tile='{}' coord=({}, {}) sites=[{}]",
                tile.tile_type->name, tile.coord.x, tile.coord.y, site_list);
            ++related_tile_reports;
        }
        for (size_t site_index = 0; site_index < tile.tile_type->sites.size(); ++site_index) {
            ++sites_seen;
            if (siteOccupied(tile, static_cast<int>(site_index))) {
                continue;
            }
            const fpga::SiteModel& site = tile.tile_type->sites[site_index];
            if (!site.type.empty()
                && std::find(sampled_site_types.begin(), sampled_site_types.end(), site.type)
                    == sampled_site_types.end()
                && sampled_site_types.size() < 16) {
                sampled_site_types.push_back(site.type);
            }
            if (!compatibleSiteType(buffer, site)) {
                continue;
            }
            ++compatible_sites;
            if (matchingPins(site, input->port_ref->makeName(), fpga::TILE_PIN_INPUT).empty()
                || matchingPins(site, output->port_ref->makeName(), fpga::TILE_PIN_OUTPUT).empty()) {
                continue;
            }
            ++pin_sites;
            int old_pos = buffer.pos;
            buffer.pos = static_cast<int>(site_index);
            buffer.tile.set(static_cast<Referable<fpga::Tile>*>(&tile));
            bool connected = !endpointNodes(device, buffer, input->port_ref->makeName(), fpga::TILE_PIN_INPUT).empty()
                && !endpointNodes(device, buffer, output->port_ref->makeName(), fpga::TILE_PIN_OUTPUT).empty();
            buffer.tile.clear();
            buffer.pos = old_pos;
            if (!connected) {
                continue;
            }
            ++endpoint_sites;
            int source_distance = std::abs(tile.coord.x - preferred.x)
                + std::abs(tile.coord.y - preferred.y);
            int64_t sink_distance = 0;
            size_t placed_sinks = 0;
            for (const ClockTask& task : fanout) {
                if (!task.to || !task.to->tile.peer) continue;
                sink_distance += std::abs(tile.coord.x - task.to->tile->coord.x)
                    + std::abs(tile.coord.y - task.to->tile->coord.y);
                ++placed_sinks;
            }
            int average_sink_distance = placed_sinks
                ? static_cast<int>(sink_distance / static_cast<int64_t>(placed_sinks)) : 0;
            int score = source_distance + average_sink_distance * 4;
            if (score < best_score) {
                best_score = score;
                best_tile = &tile;
                best_site = static_cast<int>(site_index);
            }
        }
    }
    if (!best_tile) {
        std::string samples;
        for (const std::string& site_type : sampled_site_types) {
            if (!samples.empty()) samples += ",";
            samples += site_type;
        }
        PNR_WARNING("clock buffer '{}' type='{}' placement rejected: sites={} compatible={} pins={} endpoints={} site_types=[{}]",
            buffer.makeName(), buffer.cell_ref.peer ? buffer.cell_ref.peer->type : std::string{},
            sites_seen, compatible_sites, pin_sites, endpoint_sites, samples);
        return false;
    }
    buffer.pos = best_site;
    buffer.coord = best_tile->coord;
    best_tile->assign(&buffer);
    ++stats.buffers_placed;
    return true;
}

// Route a large clock fanout from one shared numeric reachability search.
// This avoids restarting a full fabric traversal for every clock sink.
bool routeTaskTreeBatch(fpga::Device& device,
                        const std::vector<ClockTask>& tasks,
                        pnr::RouteClocks::Stats& stats,
                        pnr::RouteClocks::Failure& failure)
{
    if (tasks.empty() || !tasks.front().from) {
        return true;
    }
    for (const ClockTask& task : tasks) {
        if (task.net) {
            task.net->route_protected = true;
        }
    }

    const ClockTask& first = tasks.front();
    std::vector<Endpoint> roots = endpointNodes(
        device, *first.from, first.from_port, fpga::TILE_PIN_OUTPUT);
    std::vector<std::vector<Endpoint>> task_targets;
    task_targets.reserve(tasks.size());
    bool all_routed = true;
    for (const ClockTask& task : tasks) {
        task_targets.push_back(endpointNodes(
            device, *task.to, task.to_port, fpga::TILE_PIN_INPUT));
        if (!task_targets.back().empty()) {
            continue;
        }
        if (!failure) {
            failure.net_name = task.net_name;
            if (task.to && task.to->tile.peer) {
                failure.x = task.to->tile->coord.x;
                failure.y = task.to->tile->coord.y;
            }
        }
        ++stats.failed;
        all_routed = false;
    }
    if (roots.empty()) {
        if (!failure) {
            failure.net_name = first.net_name;
            if (first.from->tile.peer) {
                failure.x = first.from->tile->coord.x;
                failure.y = first.from->tile->coord.y;
            }
        }
        stats.failed += tasks.size();
        return false;
    }

    using SearchParents =
        std::unordered_map<RouteNodeKey, ParentEdge, RouteNodeKeyHash>;
    std::unordered_map<RouteNodeKey, std::vector<std::pair<size_t, size_t>>,
                       RouteNodeKeyHash>
        targets_by_node;
    size_t targetable_tasks = 0;
    for (size_t task_index = 0; task_index < task_targets.size(); ++task_index) {
        if (task_targets[task_index].empty()) {
            continue;
        }
        ++targetable_tasks;
        for (size_t endpoint_index = 0;
             endpoint_index < task_targets[task_index].size();
             ++endpoint_index) {
            const Endpoint& endpoint = task_targets[task_index][endpoint_index];
            targets_by_node[nodeKey(RouteNode{endpoint.route_tile,
                                               fpga::CB_NODE_LOCAL,
                                               endpoint.local})]
                .push_back({task_index, endpoint_index});
        }
    }

    ClockTree tree;
    SearchParents search_parents;
    std::vector<int> selected_endpoint(tasks.size(), -1);
    size_t selected_count = 0;
    for (const Endpoint& root : roots) {
        RouteNode root_node{root.route_tile, fpga::CB_NODE_LOCAL, root.local};
        if (nodeBusy(root_node)) {
            continue;
        }
        SearchParents candidate_parents;
        std::vector<int> candidate_selected(tasks.size(), -1);
        size_t candidate_count = 0;
        std::deque<RouteNode> queue;
        queue.push_back(root_node);
        int root_visits = -1;
        if (!tree.tile_visits.append(-1, root_node.tile, root_visits)) {
            continue;
        }
        candidate_parents.emplace(nodeKey(root_node),
                                  ParentEdge{{}, false, root_visits});
        leaseNode(root_node);
        while (!queue.empty() && candidate_count < targetable_tasks) {
            RouteNode current = queue.front();
            queue.pop_front();
            ++stats.graph_nodes;
            RouteNodeKey current_key = nodeKey(current);
            auto targets = targets_by_node.find(current_key);
            if (targets != targets_by_node.end()) {
                for (const auto& [task_index, endpoint_index] : targets->second) {
                    if (candidate_selected[task_index] < 0) {
                        candidate_selected[task_index] =
                            static_cast<int>(endpoint_index);
                        ++candidate_count;
                    }
                }
            }
            for (const RouteNode& next : outgoing(device, current)) {
                RouteNodeKey next_key = nodeKey(next);
                if (candidate_parents.contains(next_key) || nodeBusy(next)) {
                    continue;
                }
                int visit_root = candidate_parents.at(current_key).tile_visits_root;
                if (next.tile != current.tile &&
                    !tree.tile_visits.append(visit_root, next.tile, visit_root)) {
                    ++stats.tile_visit_rejects;
                    continue;
                }
                candidate_parents.emplace(
                    next_key, ParentEdge{current, true, visit_root});
                queue.push_back(next);
            }
        }
        if (candidate_selected.front() >= 0) {
            tree.root = root_node;
            tree.root_endpoint = root;
            tree.nodes.insert(nodeKey(root_node));
            tree.parents.emplace(nodeKey(root_node),
                                 ParentEdge{{}, false, root_visits});
            search_parents = std::move(candidate_parents);
            selected_endpoint = std::move(candidate_selected);
            selected_count = candidate_count;
            break;
        }
        releaseNode(root_node);
    }
    if (tree.nodes.empty()) {
        if (!failure) {
            failure.net_name = first.net_name;
            if (!task_targets.front().empty()
                && task_targets.front().front().route_tile) {
                failure.x = task_targets.front().front().route_tile->coord.x;
                failure.y = task_targets.front().front().route_tile->coord.y;
                failure.node_type = fpga::CB_NODE_LOCAL;
                failure.node = task_targets.front().front().local;
            }
        }
        stats.failed += targetable_tasks;
        return false;
    }
    tree.root_endpoint.resource_tile->pin_state.lease(tree.root_endpoint.local);

    auto merge_path = [&](const RouteNode& sink) {
        RouteNode cursor = sink;
        std::vector<RouteNode> added;
        while (!tree.nodes.contains(nodeKey(cursor))) {
            auto parent = search_parents.find(nodeKey(cursor));
            if (parent == search_parents.end() || !parent->second.valid) {
                return false;
            }
            added.push_back(cursor);
            cursor = parent->second.parent;
        }
        std::reverse(added.begin(), added.end());
        for (const RouteNode& child : added) {
            const ParentEdge& parent = search_parents.at(nodeKey(child));
            tree.parents[nodeKey(child)] = parent;
            tree.nodes.insert(nodeKey(child));
            leaseNode(child);
        }
        return true;
    };

    for (size_t task_index = 0; task_index < tasks.size(); ++task_index) {
        if (task_targets[task_index].empty()) {
            continue;
        }
        int endpoint_index = selected_endpoint[task_index];
        if (endpoint_index < 0) {
            const ClockTask& task = tasks[task_index];
            if (!failure) {
                failure.net_name = task.net_name;
                failure.x = task_targets[task_index].front().route_tile->coord.x;
                failure.y = task_targets[task_index].front().route_tile->coord.y;
                failure.node_type = fpga::CB_NODE_LOCAL;
                failure.node = task_targets[task_index].front().local;
            }
            ++stats.failed;
            all_routed = false;
            continue;
        }
        const ClockTask& task = tasks[task_index];
        const Endpoint& endpoint =
            task_targets[task_index][static_cast<size_t>(endpoint_index)];
        RouteNode sink{endpoint.route_tile, fpga::CB_NODE_LOCAL, endpoint.local};
        if (!merge_path(sink)) {
            if (!failure) {
                failure.net_name = task.net_name;
                failure.x = sink.tile->coord.x;
                failure.y = sink.tile->coord.y;
                failure.node_type = fpga::CB_NODE_LOCAL;
                failure.node = sink.value;
            }
            ++stats.failed;
            all_routed = false;
            continue;
        }
        endpoint.resource_tile->pin_state.lease(endpoint.local);
        std::vector<fpga::Wire> route = materializePath(
            tree, endpoint, *task.from, task.from_port, *task.to,
            task.to_port, task.net_name);
        if (route.empty()) {
            ++stats.failed;
            all_routed = false;
            continue;
        }
        size_t route_index = task.to->wires.size();
        task.to->wires.push_back(std::move(route));
        if (task.net) {
            size_t binding_index = fpga::attachNetRoute(
                *task.net, *task.to, route_index, task.from, task.to,
                task.from_port, task.to_port, task.net_name);
            fpga::registerNetRouteTiles(*task.net, task.to->wires.back(),
                                        binding_index);
        }
        ++stats.routed;
    }
    PNR_LOG("CLKS",
        "clock batch tree '{}' tasks={} reachable={} tree_nodes={} graph_nodes={}",
        first.net_name, tasks.size(), selected_count, tree.nodes.size(),
        search_parents.size());
    return all_routed;
}

bool routeTaskTree(fpga::Device& device, const std::vector<ClockTask>& tasks,
                   pnr::RouteClocks::Stats& stats,
                   pnr::RouteClocks::Failure& failure)
{
    constexpr size_t batch_threshold = 32;
    if (tasks.size() >= batch_threshold) {
        return routeTaskTreeBatch(device, tasks, stats, failure);
    }
    if (tasks.empty() || !tasks.front().from) {
        return true;
    }
    const ClockTask& first = tasks.front();
    if (first.net) {
        first.net->route_protected = true;
    }
    auto record_failure = [&](const ClockTask& task,
                              const std::vector<Endpoint>& endpoints) {
        if (failure) {
            return;
        }
        failure.net_name = task.net_name;
        if (!endpoints.empty() && endpoints.front().route_tile) {
            failure.x = endpoints.front().route_tile->coord.x;
            failure.y = endpoints.front().route_tile->coord.y;
            failure.node_type = fpga::CB_NODE_LOCAL;
            failure.node = endpoints.front().local;
        } else if (task.to && task.to->tile.peer) {
            failure.x = task.to->tile->coord.x;
            failure.y = task.to->tile->coord.y;
        }
    };
    std::vector<Endpoint> roots = endpointNodes(device, *first.from, first.from_port,
                                                fpga::TILE_PIN_OUTPUT);
    if (roots.empty()) {
        record_failure(first, {});
        PNR_WARNING("clock tree '{}' has no numeric source endpoint for '{}.{}'",
            first.net_name, first.from->makeName(), first.from_port);
        ++stats.failed;
        return false;
    }
    std::vector<Endpoint> first_targets = endpointNodes(
        device, *first.to, first.to_port, fpga::TILE_PIN_INPUT);
    std::string first_target_text;
    for (const Endpoint& target : first_targets) {
        if (!first_target_text.empty()) first_target_text += ", ";
        first_target_text += describeNode(RouteNode{target.route_tile, fpga::CB_NODE_LOCAL,
                                                     target.local});
    }
    ClockTree tree;
    std::optional<RouteNode> first_reached;
    std::string root_attempts;
    for (const Endpoint& root : roots) {
        RouteNode root_node{root.route_tile, fpga::CB_NODE_LOCAL, root.local};
        if (!root_attempts.empty()) root_attempts += ", ";
        root_attempts += describeNode(root_node);
        if (nodeBusy(root_node)) {
            continue;
        }
        ClockTree candidate;
        candidate.root_endpoint = root;
        candidate.root = root_node;
        candidate.nodes.insert(nodeKey(root_node));
        int root_visits = -1;
        if (!candidate.tile_visits.append(-1, root_node.tile, root_visits)) {
            continue;
        }
        candidate.parents.emplace(nodeKey(root_node),
                                  ParentEdge{{}, false, root_visits});
        leaseNode(root_node);
        std::optional<RouteNode> reached = extendTree(
            device, candidate, first_targets, stats.graph_nodes,
            stats.tile_visit_rejects);
        if (reached) {
            tree = std::move(candidate);
            first_reached = reached;
            break;
        }
        releaseNode(root_node);
    }
    if (!first_reached) {
        record_failure(first, first_targets);
        std::string target_inputs;
        for (const Endpoint& target : first_targets) {
            if (!target.route_tile || !target.route_tile->cb_type) continue;
            const fpga::CBType& type = *target.route_tile->cb_type;
            const NodeMask& dsts = type.dsts_reaching_local[target.local].jump;
            dsts.for_each_set_bit([&](int dst) {
                if (!target_inputs.empty()) target_inputs += ", ";
                RouteNode input{target.route_tile, fpga::CB_NODE_DST, dst};
                target_inputs += describeNode(input);
                target_inputs += nodeBusy(input) ? "(busy)" : "(free)";
                return false;
            });
        }
        PNR_WARNING("clock tree '{}' cannot connect any source endpoint; roots=[{}] targets=[{}] sink='{}.{}'",
            first.net_name, root_attempts, first_target_text,
            first.to ? first.to->makeName() : std::string{}, first.to_port);
        PNR_LOG("CLKS", "unreachable clock target incoming_dsts=[{}]", target_inputs);
        ++stats.failed;
        return false;
    }
    tree.root_endpoint.resource_tile->pin_state.lease(tree.root_endpoint.local);
    PNR_LOG("CLKS", "clock tree '{}' roots={} selected={} outgoing={}", first.net_name,
        roots.size(), describeNode(tree.root), outgoing(device, tree.root).size());

    bool all_routed = true;
    for (size_t task_index = 0; task_index < tasks.size(); ++task_index) {
        const ClockTask& task = tasks[task_index];
        if (task.net) {
            task.net->route_protected = true;
        }
        std::vector<Endpoint> targets = task_index == 0 ? first_targets : endpointNodes(
            device, *task.to, task.to_port, fpga::TILE_PIN_INPUT);
        if (targets.empty()) {
            record_failure(task, targets);
            PNR_WARNING("clock sink '{}' has no numeric endpoint for '{}.{}'",
                task.net_name, task.to->makeName(), task.to_port);
        }
        size_t visited_before = stats.graph_nodes;
        std::optional<RouteNode> reached = task_index == 0 ? first_reached
            : extendTree(device, tree, targets, stats.graph_nodes,
                         stats.tile_visit_rejects);
        if (!reached) {
            record_failure(task, targets);
            static size_t reported_failures = 0;
            if (reported_failures++ < 5) {
                std::string target_text;
                for (const Endpoint& target : targets) {
                    if (!target_text.empty()) target_text += ", ";
                    target_text += describeNode(RouteNode{target.route_tile, fpga::CB_NODE_LOCAL,
                                                          target.local});
                }
                PNR_WARNING("clock route '{}' failed from tree_nodes={} after visiting {} nodes; targets=[{}]",
                    task.net_name, tree.nodes.size(), stats.graph_nodes - visited_before, target_text);
            }
            ++stats.failed;
            all_routed = false;
            continue;
        }
        const Endpoint* endpoint = selectedEndpoint(targets, *reached);
        if (!endpoint) {
            record_failure(task, targets);
            ++stats.failed;
            all_routed = false;
            continue;
        }
        endpoint->resource_tile->pin_state.lease(endpoint->local);
        std::vector<fpga::Wire> route = materializePath(
            tree, *endpoint, *task.from, task.from_port, *task.to, task.to_port, task.net_name);
        if (route.empty()) {
            record_failure(task, targets);
            ++stats.failed;
            all_routed = false;
            continue;
        }
        size_t route_index = task.to->wires.size();
        task.to->wires.push_back(std::move(route));
        if (task.net) {
            size_t binding_index = fpga::attachNetRoute(
                *task.net, *task.to, route_index, task.from, task.to,
                task.from_port, task.to_port, task.net_name);
            fpga::registerNetRouteTiles(*task.net, task.to->wires.back(),
                                        binding_index);
        }
        ++stats.routed;
    }
    return all_routed;
}

}

pnr::RouteClocks::RouteClocks(technology::Tech& tech, fpga::Device& device)
    : tech_(tech), device_(device)
{
}

bool pnr::RouteClocks::routeDesign(clk::Clocks& clocks)
{
    stats_ = {};
    failure_ = {};
    device_.activateDeferredCBTypes();
    std::vector<rtl::Inst*> leaves;
    collectLeafInsts(tech_.design.top, leaves);
    bool success = true;

    for (auto& clock_ref : clocks.clocks_list) {
        rtl::Clock& clock = clock_ref;
        ++stats_.clocks;
        rtl::Inst* buffer = clock.bufg_ptr;
        if (!buffer) {
            if (!failure_) failure_.net_name = clock.name;
            PNR_WARNING("clock '{}' has no placeable routed buffer endpoint", clock.name);
            ++stats_.failed;
            success = false;
            continue;
        }

        rtl::Conn* buffer_input = nullptr;
        rtl::Conn* buffer_output = nullptr;
        for (rtl::Conn& conn : buffer->conns) {
            if (!conn.port_ref.peer) continue;
            if (conn.port_ref->type == rtl::Port::PORT_IN && !buffer_input) buffer_input = &conn;
            if (conn.port_ref->type == rtl::Port::PORT_OUT && !buffer_output) buffer_output = &conn;
        }
        if (!buffer_input || !buffer_output) {
            if (!failure_) failure_.net_name = clock.name;
            ++stats_.failed;
            success = false;
            continue;
        }

        std::vector<ClockTask> fanout;
        for (rtl::Inst* sink : leaves) {
            if (!sink || sink == buffer || !sink->cell_ref.peer) {
                continue;
            }
            for (rtl::Conn& conn : sink->conns) {
                if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN
                    || !tech_.check_clocked(sink->cell_ref->type, conn.port_ref->name)) {
                    continue;
                }
                rtl::Conn* driver = conn.follow();
                if (!driver || driver->inst_ref.peer != buffer) {
                    continue;
                }
                fanout.push_back(ClockTask{buffer, sink, findNetByDesignator(*sink, conn.port_ref->designator),
                    buffer_output->port_ref->makeName(), conn.port_ref->makeName(), conn.makeNetName()});
            }
        }
        if (!placeClockBuffer(device_, *buffer, fanout, stats_)) {
            if (!failure_) failure_.net_name = clock.name;
            PNR_WARNING("clock '{}' has no placeable routed buffer endpoint", clock.name);
            ++stats_.failed;
            success = false;
            continue;
        }

        rtl::Conn* input_driver = buffer_input->follow();
        if (input_driver && input_driver->inst_ref.peer) {
            ClockTask input_task{input_driver->inst_ref.peer, buffer,
                                 findNetByDesignator(*buffer, buffer_input->port_ref->designator),
                                 input_driver->port_ref->makeName(), buffer_input->port_ref->makeName(),
                                 buffer_input->makeNetName()};
            ++stats_.nets;
            ++stats_.sinks;
            success = routeTaskTree(device_, {input_task}, stats_, failure_) && success;
        }

        if (!fanout.empty()) {
            ++stats_.nets;
            stats_.sinks += fanout.size();
            success = routeTaskTree(device_, fanout, stats_, failure_) && success;
        }
    }
    PNR_LOG("CLKS", "clock routing clocks={} buffers_placed={} nets={} sinks={} routed={} failed={} graph_nodes={} tile_visit_rejects={}",
        stats_.clocks, stats_.buffers_placed, stats_.nets, stats_.sinks,
        stats_.routed, stats_.failed, stats_.graph_nodes,
        stats_.tile_visit_rejects);
    return success && stats_.failed == 0;
}
