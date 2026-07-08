#include "Device.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <tuple>
#include <unordered_set>
#include <unordered_map>

using namespace fpga;

namespace technology {
#if defined(__GNUC__)
std::vector<std::pair<std::string, std::string>> mappedRouteEndpointAliases(
    const std::string& tile_type, const std::string& pin, int site_pos, const std::string& wire) __attribute__((weak));
#else
std::vector<std::pair<std::string, std::string>> mappedRouteEndpointAliases(
    const std::string& tile_type, const std::string& pin, int site_pos, const std::string& wire);
#endif
}

namespace {

TileType* tileTypeFor(std::vector<TileType>& tile_types, const std::string& name)
{
    for (TileType& type : tile_types) {
        if (type.name == name) {
            return &type;
        }
    }
    return nullptr;
}

void addResolvedJumpBit(std::vector<CBType::ResolvedJump>& entries, Coord delta,
                        uint16_t target_cb_type_id, int dst_node,
                        const std::string& source_cb_type, int src_node,
                        const std::string& target_cb_type,
                        const std::string& dst_wire = std::string{},
                        bool target_tile_coord = false)
{
    (void)source_cb_type;
    (void)src_node;
    (void)target_cb_type;
    if (target_cb_type_id == CB_INVALID_TYPE_ID || dst_node < 0 || dst_node >= CB_MAX_NODES) {
        return;
    }
    NodeMask bit = NodeMask{0,1} << dst_node;
    for (CBType::ResolvedJump& entry : entries) {
        if (entry.target_cb_type_id != target_cb_type_id) {
            continue;
        }
        if (entry.delta.x == delta.x && entry.delta.y == delta.y
            && entry.target_tile_coord == target_tile_coord) {
            entry.dsts.jump |= bit;
            if (!dst_wire.empty()) {
                entry.dst_wires[static_cast<uint16_t>(dst_node)] = dst_wire;
            }
            return;
        }
    }
    CBJumpState dsts{};
    dsts.jump = bit;
    CBType::ResolvedJump entry{delta, target_cb_type_id, dsts, {}, target_tile_coord};
    if (!dst_wire.empty()) {
        entry.dst_wires[static_cast<uint16_t>(dst_node)] = dst_wire;
    }
    entries.push_back(std::move(entry));
}

int jumpIndexForDeltaLocal(Coord delta, int num);
Coord jumpDeltaFromNodeLocal(int node);

void addSrcPriorityDelta(CBType& cb_type, int src_node, Coord delta)
{
    if (src_node < 0 || src_node >= CB_MAX_NODES || (delta.x == 0 && delta.y == 0)) {
        return;
    }
    if (delta.x < -8 || delta.x > 7 || delta.y < -8 || delta.y > 7) {
        delta = jumpDeltaFromNodeLocal(src_node);
        if (delta.x == 0 && delta.y == 0) {
            return;
        }
    }
    auto& deltas = cb_type.src_priority_deltas[static_cast<uint16_t>(src_node)];
    if (deltas.empty()) {
        NodeMask src_bit = NodeMask{0,1} << src_node;
        for (auto& [_, srcs] : cb_type.priority_srcs_by_delta.values) {
            srcs.jump &= ~src_bit;
        }
    }
    for (const Coord& existing : deltas) {
        if (existing.x == delta.x && existing.y == delta.y) {
            return;
        }
    }
    deltas.push_back(delta);
    cb_type.priority_srcs_by_delta[jumpIndexForDeltaLocal(delta, 0)].jump |= NodeMask{0,1} << src_node;
}

int encodeSigned4Local(int value)
{
    PNR_ASSERT(value >= -8 && value <= 7, "jump delta {} is outside signed 4-bit encoding", value);
    return value & 0xf;
}

int decodeSigned4Local(int value)
{
    value &= 0xf;
    return (value & 0x8) ? value - 16 : value;
}

int jumpIndexForDeltaLocal(Coord delta, int num)
{
    PNR_ASSERT(num >= 0 && num < 16, "jump lane {} is outside 4-bit .num encoding", num);
    return (encodeSigned4Local(delta.x) << 8) | (encodeSigned4Local(delta.y) << 4) | (num & 0xf);
}

Coord jumpDeltaFromNodeLocal(int node)
{
    return Coord{decodeSigned4Local((node >> 8) & 0xf),
                 decodeSigned4Local((node >> 4) & 0xf)};
}

bool signed4DeltaInRange(Coord delta)
{
    return delta.x >= -8 && delta.x <= 7 && delta.y >= -8 && delta.y <= 7;
}

NodeMask jumpNodesStructurallyUsedAs(const CBType& cb_type, CBNodeNameType role)
{
    NodeMask used;
    if (role == CB_NODE_SRC) {
        for (const auto& [key, name] : cb_type.node_names) {
            (void)name;
            if (key.type == CB_NODE_SRC) {
                used |= NodeMask{0,1} << key.value;
            }
        }
        for (int pos = 0; pos < CB_MAX_NODES; ++pos) {
            if (cb_type.src_joint[pos].joint != NodeMask{} || !cb_type.dst_by_src[pos].empty()) {
                used |= NodeMask{0,1} << pos;
            }
            used |= cb_type.local_src[pos].jump;
            used |= cb_type.joint_src[pos].jump;
            used |= cb_type.dst_src[pos].jump;
        }
        return used;
    }
    if (role == CB_NODE_DST) {
        for (int pos = 0; pos < CB_MAX_NODES; ++pos) {
            if ((cb_type.dst_src[pos].jump | cb_type.dst_joint[pos].joint) != NodeMask{}) {
                used |= NodeMask{0,1} << pos;
            }
            used |= cb_type.dstMaskForSrc(pos);
        }
    }
    return used;
}

NodeMask jumpSourcesUsedByRouteMasks(const CBType& cb_type)
{
    NodeMask used;
    for (int pos = 0; pos < CB_MAX_NODES; ++pos) {
        if (cb_type.src_joint[pos].joint != NodeMask{} || !cb_type.dst_by_src[pos].empty()) {
            used |= NodeMask{0,1} << pos;
        }
        used |= cb_type.local_src[pos].jump;
        used |= cb_type.joint_src[pos].jump;
        used |= cb_type.dst_src[pos].jump;
    }
    return used;
}

int countBits(const NodeMask& mask)
{
    int count = 0;
    mask.for_each_set_bit([&](int) {
        ++count;
        return false;
    });
    return count;
}

bool debugResolveJumpCoord(const Coord& coord)
{
    const char* text = std::getenv("SCALEPNR_DEBUG_RESOLVE_JUMP");
    if (!text || !*text) {
        return false;
    }
    int x = 0;
    int y = 0;
    if (std::sscanf(text, "%d,%d", &x, &y) != 2) {
        return false;
    }
    return coord.x == x && coord.y == y;
}

bool debugTileConnCoord(const Coord& coord)
{
    const char* text = std::getenv("SCALEPNR_DEBUG_TILECONN_COORD");
    if (!text || !*text) {
        return false;
    }
    int x = 0;
    int y = 0;
    if (std::sscanf(text, "%d,%d", &x, &y) != 2) {
        return false;
    }
    return coord.x == x && coord.y == y;
}

bool debugTileConnWire(const std::string& wire)
{
    const char* text = std::getenv("SCALEPNR_DEBUG_TILECONN_WIRE");
    return text && *text && wire.find(text) != std::string::npos;
}

bool debugEndpointAlias(const std::string& tile_type, const std::string& pin)
{
    const char* text = std::getenv("SCALEPNR_DEBUG_ENDPOINT_ALIAS");
    if (!text || !*text) {
        return false;
    }
    return tile_type.find(text) != std::string::npos || pin.find(text) != std::string::npos;
}

bool debugEndpointWire(const std::string& tile_type, const std::string& wire)
{
    const char* text = std::getenv("SCALEPNR_DEBUG_ENDPOINT_WIRE");
    if (!text || !*text) {
        return false;
    }
    return tile_type.find(text) != std::string::npos || wire.find(text) != std::string::npos;
}

std::optional<std::string> routeWireFamilyKey(const std::string& wire)
{
    for (size_t start = 0; start < wire.size(); ++start) {
        if (wire[start] != 'N' && wire[start] != 'S'
            && wire[start] != 'E' && wire[start] != 'W') {
            continue;
        }
        size_t pos = start;
        while (pos < wire.size() && (wire[pos] == 'N' || wire[pos] == 'S'
            || wire[pos] == 'E' || wire[pos] == 'W')) {
            ++pos;
        }
        while (pos < wire.size() && std::isdigit(static_cast<unsigned char>(wire[pos]))) {
            ++pos;
        }
        if (pos == start || pos >= wire.size()) {
            continue;
        }

        std::string prefix = wire.substr(start, pos - start);
        if (wire.compare(pos, 3, "BEG") == 0 || wire.compare(pos, 3, "END") == 0) {
            pos += 3;
        }
        else if (wire[pos] >= 'A' && wire[pos] <= 'E') {
            ++pos;
        }
        else {
            continue;
        }
        return prefix + "|" + wire.substr(pos);
    }
    return std::nullopt;
}

bool sameRouteWireFamily(const std::optional<std::string>& family, const std::string& wire)
{
    if (!family) {
        return true;
    }
    std::optional<std::string> other = routeWireFamilyKey(wire);
    return other && *other == *family;
}

std::string tileConnKey(const std::string& tile_type, const std::string& wire);
bool cbHasRouteGraph(CBType& cb_type);

void pruneUnresolvedRouteSources(CBType& cb_type)
{
    NodeMask unresolved_srcs;
    NodeMask used_srcs = jumpSourcesUsedByRouteMasks(cb_type);
    used_srcs.for_each_set_bit([&](int src_node) {
        if (cb_type.dst_by_src[src_node].empty()) {
            unresolved_srcs |= NodeMask{0,1} << src_node;
            cb_type.src_joint[src_node] = {};
        }
        return false;
    });
    if (unresolved_srcs == NodeMask{}) {
        return;
    }
    for (auto& [node, state] : cb_type.local_src.values) {
        (void)node;
        state.jump &= ~unresolved_srcs;
    }
    for (auto& [node, state] : cb_type.dst_src.values) {
        (void)node;
        state.jump &= ~unresolved_srcs;
    }
    for (auto& [node, state] : cb_type.joint_src.values) {
        (void)node;
        state.jump &= ~unresolved_srcs;
    }
    cb_type.derived_masks_valid = false;
    cb_type.rebuildOutgoingSrcs();
}

bool siteTypeHasPackableElement(const std::string& type)
{
    return type.find("LUT") != std::string::npos
        || type.find("FD") != std::string::npos
        || type.find("CARRY") != std::string::npos
        || type.find("MUX") != std::string::npos
        || type.find("SLICE") != std::string::npos;
}

bool typeSpecHasPackableSite(const TypeSpec& type)
{
    return std::any_of(type.sites.begin(), type.sites.end(), [](const TypeSpec::SiteSpec& site) {
        return siteTypeHasPackableElement(site.type);
    });
}

std::string tileTypePrefix(const std::string& tile_name)
{
    size_t pos = tile_name.rfind("_X");
    return pos == std::string::npos ? tile_name : tile_name.substr(0, pos);
}

bool sameVendorTileCoord(const std::string& a, const std::string& b)
{
    size_t ax = a.rfind("_X");
    size_t bx = b.rfind("_X");
    if (ax == std::string::npos || bx == std::string::npos) {
        return false;
    }
    return a.substr(ax) == b.substr(bx);
}

struct ResolveJumpCacheKey
{
    const Tile* from = nullptr;
    const CBType* cb_type = nullptr;
    int src_node = -1;

    bool operator==(const ResolveJumpCacheKey& other) const
    {
        return from == other.from && cb_type == other.cb_type && src_node == other.src_node;
    }
};

struct ResolveJumpCacheKeyHash
{
    std::size_t operator()(const ResolveJumpCacheKey& key) const
    {
        std::size_t ptr = reinterpret_cast<std::size_t>(key.from);
        std::size_t cb_ptr = reinterpret_cast<std::size_t>(key.cb_type);
        std::size_t hash = (ptr >> 4) ^ (cb_ptr >> 6)
            ^ (static_cast<std::size_t>(key.src_node) << 1);
        return hash;
    }
};

struct ResolveJumpTowardCacheKey
{
    ResolveJumpCacheKey base;
    int target_x = 0;
    int target_y = 0;

    bool operator==(const ResolveJumpTowardCacheKey& other) const
    {
        return base == other.base && target_x == other.target_x && target_y == other.target_y;
    }
};

struct ResolveJumpTowardCacheKeyHash
{
    std::size_t operator()(const ResolveJumpTowardCacheKey& key) const
    {
        std::size_t hash = ResolveJumpCacheKeyHash{}(key.base);
        hash ^= static_cast<std::size_t>(key.target_x) + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
        hash ^= static_cast<std::size_t>(key.target_y) + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
        return hash;
    }
};

struct AttachedCbKey
{
    uint16_t base_type_id = CB_INVALID_TYPE_ID;
    int x = 0;
    int y = 0;

    bool operator==(const AttachedCbKey& other) const
    {
        return base_type_id == other.base_type_id && x == other.x && y == other.y;
    }
};

struct AttachedCbKeyHash
{
    std::size_t operator()(const AttachedCbKey& key) const
    {
        return static_cast<std::size_t>(key.base_type_id)
            ^ (static_cast<std::size_t>(key.x) << 20)
            ^ static_cast<std::size_t>(key.y);
    }
};

void hashCombine(std::size_t& seed, std::size_t value)
{
    seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
}

std::size_t nodeMaskHash(const NodeMask& mask)
{
    std::size_t hash = 0;
    for (int i = 0; i < NodeMask::words_count; ++i) {
        hashCombine(hash, static_cast<std::size_t>(static_cast<uint64_t>(mask.words[i])));
        hashCombine(hash, static_cast<std::size_t>(static_cast<uint64_t>(mask.words[i] >> 64)));
    }
    return hash;
}

std::size_t resolveJumpTopologyHash(const CBType& cb_type, int src_node)
{
    std::size_t hash = nodeMaskHash(cb_type.dstMaskForSrc(src_node));
    hashCombine(hash, cb_type.dst_by_src[src_node].size());
    for (const CBType::ResolvedJump& entry : cb_type.dst_by_src[src_node]) {
        hashCombine(hash, static_cast<std::size_t>(static_cast<uint16_t>(entry.delta.x)));
        hashCombine(hash, static_cast<std::size_t>(static_cast<uint16_t>(entry.delta.y)));
        hashCombine(hash, entry.target_cb_type_id);
        hashCombine(hash, nodeMaskHash(entry.dsts.jump));
        hashCombine(hash, entry.dst_wires.size());
        for (const auto& [dst, wire] : entry.dst_wires) {
            hashCombine(hash, dst);
            hashCombine(hash, std::hash<std::string>{}(wire));
        }
        hashCombine(hash, entry.target_tile_coord ? 1 : 0);
    }
    return hash;
}

CBType* exactCBTypeFor(std::vector<CBType>& cb_types, const std::string& tile_type_name)
{
    for (CBType& cb_type : cb_types) {
        if (cb_type.name == tile_type_name) {
            return &cb_type;
        }
    }
    return nullptr;
}

const CBType* exactCBTypeFor(const std::vector<CBType>& cb_types, const std::string& tile_type_name)
{
    for (const CBType& cb_type : cb_types) {
        if (cb_type.name == tile_type_name) {
            return &cb_type;
        }
    }
    return nullptr;
}

uint16_t cbTypeIdFor(const std::vector<CBType>& cb_types, const CBType* cb_type)
{
    if (!cb_type) {
        return CB_INVALID_TYPE_ID;
    }
    PNR_ASSERT(cb_type >= cb_types.data() && cb_type < cb_types.data() + cb_types.size(),
        "CBType pointer '{}' is not owned by Device::cb_types\n", cb_type->name);
    size_t index = static_cast<size_t>(cb_type - cb_types.data());
    PNR_ASSERT(index < CB_INVALID_TYPE_ID, "too many CB types: {}\n", index);
    return static_cast<uint16_t>(index);
}

const CBType* cbTypeById(const std::vector<CBType>& cb_types, uint16_t cb_type_id)
{
    if (cb_type_id == CB_INVALID_TYPE_ID || cb_type_id >= cb_types.size()) {
        return nullptr;
    }
    return &cb_types[cb_type_id];
}

uint16_t cbBaseTypeId(const CBType* cb_type)
{
    if (!cb_type) {
        return CB_INVALID_TYPE_ID;
    }
    return cb_type->base_type_id != CB_INVALID_TYPE_ID ? cb_type->base_type_id : cb_type->type_id;
}

std::vector<std::string> physicalWireEndpointAliases(const std::string& name, CBNodeNameType node_type)
{
    std::vector<std::string> aliases;
    size_t suffix = name.size();
    while (suffix > 0 && std::isdigit(static_cast<unsigned char>(name[suffix - 1]))) {
        --suffix;
    }
    if (suffix == 0 || suffix == name.size()
        || !std::isalpha(static_cast<unsigned char>(name[suffix - 1]))) {
        return aliases;
    }

    std::string prefix = name.substr(0, suffix - 1);
    std::string tail = name.substr(suffix);
    if (node_type == CB_NODE_DST) {
        aliases.push_back(prefix + "END" + tail);
        aliases.push_back(prefix + "BEG" + tail);
    }
    else if (node_type == CB_NODE_SRC) {
        aliases.push_back(prefix + "BEG" + tail);
        aliases.push_back(prefix + "END" + tail);
    }
    else {
        aliases.push_back(prefix + "BEG" + tail);
        aliases.push_back(prefix + "END" + tail);
    }
    return aliases;
}

int nodeNumByPhysicalWireNameImpl(const CBType& cb_type, CBNodeNameType node_type,
                                  const std::string& wire, bool allow_endpoint_aliases)
{
    auto try_name = [&](const std::string& name) {
        int node = cb_type.nodeNum(node_type, name);
        if (node >= 0) {
            return node;
        }
        if (allow_endpoint_aliases) {
            for (const std::string& alias : physicalWireEndpointAliases(name, node_type)) {
                node = cb_type.nodeNum(node_type, alias);
                if (node >= 0) {
                    return node;
                }
            }
        }
        return -1;
    };

    int node = try_name(wire);
    if (node >= 0) {
        return node;
    }

    size_t dot = wire.rfind('.');
    if (dot != std::string::npos) {
        node = try_name(wire.substr(dot + 1));
        if (node >= 0) {
            return node;
        }
    }

    std::string type_prefix = cb_type.name + "_";
    if (wire.rfind(type_prefix, 0) == 0) {
        node = try_name(wire.substr(type_prefix.size()));
        if (node >= 0) {
            return node;
        }
    }

    for (size_t pos = wire.find('_'); pos != std::string::npos; pos = wire.find('_', pos + 1)) {
        node = try_name(wire.substr(pos + 1));
        if (node >= 0) {
            return node;
        }
    }
    return -1;
}

int nodeNumByPhysicalWireName(const CBType& cb_type, CBNodeNameType node_type, const std::string& wire)
{
    return nodeNumByPhysicalWireNameImpl(cb_type, node_type, wire, true);
}

int exactRouteDstNodeByPhysicalWireName(const CBType& cb_type, const std::string& wire)
{
    return nodeNumByPhysicalWireNameImpl(cb_type, CB_NODE_DST, wire, true);
}

int switchableRouteDstNodeByPhysicalWireName(const CBType& cb_type, const std::string& wire)
{
    int dst = nodeNumByPhysicalWireNameImpl(cb_type, CB_NODE_DST, wire, false);
    if (dst < 0) {
        return -1;
    }
    if (cb_type.dst_src[dst].jump == NodeMask{}
        && cb_type.dst_joint[dst].joint == NodeMask{}
        && cb_type.dst_local[dst].local == NodeMask{}) {
        return -1;
    }
    return dst;
}

int routeSrcNodeByPhysicalWireName(CBType& cb_type, const std::string& wire)
{
    int src = nodeNumByPhysicalWireNameImpl(cb_type, CB_NODE_SRC, wire, false);
    if (src >= 0) {
        return src;
    }
    return -1;
}

int routeDstNodeByPhysicalWireName(CBType& cb_type, const std::string& wire)
{
    int dst = nodeNumByPhysicalWireNameImpl(cb_type, CB_NODE_DST, wire, false);
    if (dst >= 0) {
        return dst;
    }
    return -1;
}

int cbTransitScore(CBType& cb_type)
{
    cb_type.ensureDerivedMasks();
    return countBits(cb_type.valid_dst_nodes);
}

bool cbHasRouteGraph(CBType& cb_type)
{
    cb_type.ensureDerivedMasks();
    const CBType& view = cb_type;
    if (cb_type.valid_dst_nodes != NodeMask{}
        || cb_type.local_input_nodes != NodeMask{}
        || cb_type.local_output_nodes != NodeMask{}) {
        return true;
    }
    for (int pos = 0; pos < CB_MAX_NODES; ++pos) {
        if (view.local_src[pos].jump != NodeMask{}
            || view.local_joint[pos].joint != NodeMask{}
            || view.local_local[pos].local != NodeMask{}
            || view.src_joint[pos].joint != NodeMask{}
            || !view.dst_by_src[pos].empty()
            || view.joint_src[pos].jump != NodeMask{}
            || view.joint_local[pos].local != NodeMask{}
            || view.joint_joint[pos].joint != NodeMask{}
            || view.dst_src[pos].jump != NodeMask{}
            || view.dst_local[pos].local != NodeMask{}
            || view.dst_joint[pos].joint != NodeMask{}) {
            return true;
        }
    }
    return false;
}

CBType* cbTypeFor(std::vector<CBType>& cb_types,
                  const std::unordered_map<std::string, std::vector<LocalRouteWireMapping>>& local_route_wire_mappings,
                  const std::string& tile_type_name, int grid_x)
{
    static const CBType* cached_data = nullptr;
    static size_t cached_size = 0;
    static std::vector<CBType*> cached_best_types;
    static std::unordered_map<std::string, CBType*> cached_assignments;
    if (cached_data != cb_types.data() || cached_size != cb_types.size()) {
        cached_data = cb_types.data();
        cached_size = cb_types.size();
        cached_best_types.clear();
        cached_assignments.clear();
        int best_score = 0;
        for (CBType& cb_type : cb_types) {
            best_score = std::max(best_score, cbTransitScore(cb_type));
        }
        if (best_score > 0) {
            for (CBType& cb_type : cb_types) {
                if (cbTransitScore(cb_type) == best_score) {
                    cached_best_types.push_back(&cb_type);
                }
            }
        }
    }

    int fallback_slots = cached_best_types.empty() ? static_cast<int>(cb_types.size()) : static_cast<int>(cached_best_types.size());
    int fallback_slot = fallback_slots > 0 ? grid_x % fallback_slots : 0;
    std::string cache_key = tile_type_name + "\n" + std::to_string(fallback_slot);
    if (auto it = cached_assignments.find(cache_key); it != cached_assignments.end()) {
        return it->second;
    }

    auto remember_assignment = [&](CBType* cb_type) {
        cached_assignments[cache_key] = cb_type;
        return cb_type;
    };

    if (CBType* cb_type = exactCBTypeFor(cb_types, tile_type_name)) {
        if (cbHasRouteGraph(*cb_type)) {
            return remember_assignment(cb_type);
        }
    }

    const std::string key_prefix = tile_type_name + "\n";
    for (const auto& [key, mappings] : local_route_wire_mappings) {
        if (key.compare(0, key_prefix.size(), key_prefix) != 0) {
            continue;
        }
        for (const LocalRouteWireMapping& mapping : mappings) {
            if (CBType* cb_type = exactCBTypeFor(cb_types, mapping.route_type)) {
                if (cbTransitScore(*cb_type) > 0) {
                    return remember_assignment(cb_type);
                }
            }
        }
    }

    // In database-backed devices, do not invent route fabric for unmapped tile types.
    if (!local_route_wire_mappings.empty()) {
        return remember_assignment(nullptr);
    }

    if (cached_best_types.empty()) {
        return remember_assignment(cb_types.empty() ? nullptr : &cb_types[fallback_slot]);
    }
    return remember_assignment(cached_best_types[fallback_slot]);
}

std::string genericLocalNodeName(std::string wire)
{
    const std::string clbll = "CLBLL_";
    const std::string clblm = "CLBLM_";
    if (wire.compare(0, clbll.size(), clbll) == 0) {
        wire.erase(0, clbll.size());
    }
    if (wire.compare(0, clblm.size(), clblm) == 0) {
        wire.erase(0, clblm.size());
    }
    return wire;
}

std::vector<int> resolveLocalNodesInCB(const CBType& cb_type, const std::string& wire)
{
    std::string node_name = genericLocalNodeName(wire);
    std::vector<std::string> variants{node_name};
    for (const std::string prefix : {"IMUX", "BYP", "CLK", "CTRL", "FAN"}) {
        if (node_name.compare(0, prefix.length(), prefix) == 0) {
            variants.push_back(prefix + "_L" + node_name.substr(prefix.length()));
            variants.push_back(prefix + "_R" + node_name.substr(prefix.length()));
        }
    }
    std::vector<int> nodes;
    for (const std::string& variant : variants) {
        int node = cb_type.localNodeNum(variant);
        if (node >= 0 && std::find(nodes.begin(), nodes.end(), node) == nodes.end()) {
            nodes.push_back(node);
        }
    }
    return nodes;
}

struct LocalNodeMapping
{
    std::string route_type;
    int local_node = -1;
    Coord delta;
};

enum class LocalNodeUse
{
    any,
    input,
    output,
};

std::string tileConnKey(const std::string& tile_type, const std::string& wire);
std::string tileConnDeltaKey(const std::string& tile_type, const std::string& wire, Coord delta);
void addRouteWireGraphEdge(std::unordered_map<std::string, std::vector<RouteWireGraphEdge>>& graph,
                           const std::string& from_tile_type, const std::string& from_wire,
                           const std::string& to_tile_type, const std::string& to_wire,
                           Coord delta = Coord{0,0}, bool tileconn = false);

std::vector<LocalNodeMapping> resolveLocalNodeMappings(const std::vector<CBType>& cb_types,
                                                       const std::unordered_map<std::string, std::vector<LocalRouteWireMapping>>& local_route_wire_mappings,
                                                       const std::unordered_map<std::string, std::vector<RouteWireGraphEdge>>& route_wire_graph,
                                                       const std::string& tile_type_name,
                                                       const std::string& wire,
                                                       LocalNodeUse use)
{
    // Resolve a tile resource wire to concrete locals grouped by adjacent route tile type.
    std::vector<LocalNodeMapping> mappings;
    auto accepted_node = [&](const CBType& cb_type, int node) {
        if (node < 0) {
            return false;
        }
        if (use == LocalNodeUse::any) {
            return true;
        }
        const_cast<CBType&>(cb_type).ensureDerivedMasks();
        if (use == LocalNodeUse::input) {
            return cb_type.dsts_reaching_local[node].jump != NodeMask{};
        }
        return cb_type.local_src[node].jump != NodeMask{};
    };
    auto append_nodes = [&](const std::string& route_type, const CBType& cb_type, const std::string& cb_wire, Coord delta) {
        for (int node : resolveLocalNodesInCB(cb_type, cb_wire)) {
            if (!accepted_node(cb_type, node)) {
                continue;
            }
            auto same = [&](const LocalNodeMapping& mapping) {
                return mapping.route_type == route_type
                    && mapping.local_node == node
                    && mapping.delta.x == delta.x
                    && mapping.delta.y == delta.y;
            };
            if (std::find_if(mappings.begin(), mappings.end(), same) == mappings.end()) {
                mappings.push_back(LocalNodeMapping{route_type, node, delta});
            }
        }
    };

    auto mappings_it = local_route_wire_mappings.find(tileConnKey(tile_type_name, wire));
    if (mappings_it != local_route_wire_mappings.end()) {
        for (const LocalRouteWireMapping& route_mapping : mappings_it->second) {
            const CBType* cb_type = exactCBTypeFor(cb_types, route_mapping.route_type);
            if (!cb_type) {
                continue;
            }
            append_nodes(route_mapping.route_type, *cb_type, route_mapping.route_wire, route_mapping.delta);
        }
    }
    if (!mappings.empty()) {
        return mappings;
    }

    // Site endpoint wires can be one or more tile-internal pips away from the routable local.
    struct QueueItem
    {
        std::string tile_type;
        std::string wire;
        Coord delta;
        int depth = 0;
        std::string path;
    };
    bool debug_wire = debugEndpointWire(tile_type_name, wire);
    std::deque<QueueItem> queue;
    std::unordered_set<std::string> seen;
    queue.push_back(QueueItem{tile_type_name, wire, Coord{0,0}, 0, tile_type_name + ":" + wire});
    seen.insert(tileConnDeltaKey(tile_type_name, wire, Coord{0,0}));
    constexpr int max_endpoint_wire_depth = 8;
    constexpr int max_endpoint_wire_delta = 8;
    int found_depth = -1;
    while (!queue.empty()) {
        QueueItem item = std::move(queue.front());
        queue.pop_front();
        if (found_depth >= 0 && item.depth > found_depth) {
            continue;
        }
        if (const CBType* cb_type = exactCBTypeFor(cb_types, item.tile_type)) {
            size_t before = mappings.size();
            append_nodes(item.tile_type, *cb_type, item.wire, item.delta);
            if (mappings.size() != before && found_depth < 0) {
                found_depth = item.depth;
                if (debug_wire) {
                    for (size_t index = before; index < mappings.size(); ++index) {
                        const LocalNodeMapping& mapping = mappings[index];
                        const std::string* node_name = cb_type->nodeName(CB_NODE_LOCAL, mapping.local_node);
                        PNR_LOG("FPGA", "endpoint resolve found start='{}:{}' use={} local={} '{}' route='{}' delta=({}, {}) depth={} path={}",
                            tile_type_name, wire, static_cast<int>(use), mapping.local_node,
                            node_name ? *node_name : std::string{}, mapping.route_type,
                            mapping.delta.x, mapping.delta.y, item.depth, item.path);
                    }
                }
            }
        }
        if (found_depth >= 0 || item.depth >= max_endpoint_wire_depth) {
            continue;
        }
        auto edges_it = route_wire_graph.find(tileConnKey(item.tile_type, item.wire));
        if (edges_it == route_wire_graph.end()) {
            continue;
        }
        for (const RouteWireGraphEdge& edge : edges_it->second) {
            Coord next_delta{item.delta.x + edge.delta.x, item.delta.y + edge.delta.y};
            if (std::abs(next_delta.x) > max_endpoint_wire_delta
                || std::abs(next_delta.y) > max_endpoint_wire_delta) {
                continue;
            }
            std::string key = tileConnDeltaKey(edge.tile_type, edge.wire, next_delta);
            if (seen.insert(key).second) {
                std::string next_path;
                if (debug_wire) {
                    next_path = item.path + " -> " + edge.tile_type + ":" + edge.wire
                        + "(" + std::to_string(next_delta.x) + "," + std::to_string(next_delta.y) + ")";
                }
                queue.push_back(QueueItem{edge.tile_type, edge.wire, next_delta, item.depth + 1, std::move(next_path)});
            }
        }
    }
    if (!mappings.empty()) {
        return mappings;
    }

    if (use == LocalNodeUse::any) {
        for (const CBType& cb_type : cb_types) {
            append_nodes(cb_type.name, cb_type, wire, Coord{0,0});
        }
    }
    return mappings;
}

// Validate that a resource endpoint is mapped to a local node with the matching route direction.
bool localNodeMatchesUse(const std::vector<CBType>& cb_types, const LocalNodeMapping& mapping, LocalNodeUse use)
{
    if (use == LocalNodeUse::any) {
        return true;
    }
    const CBType* cb_type = exactCBTypeFor(cb_types, mapping.route_type);
    if (!cb_type || mapping.local_node < 0) {
        return false;
    }
    const_cast<CBType*>(cb_type)->ensureDerivedMasks();
    NodeMask local_bit = NodeMask{0,1} << mapping.local_node;
    if (use == LocalNodeUse::input) {
        return (cb_type->local_input_nodes & local_bit) != NodeMask{};
    }
    return (cb_type->local_output_nodes & local_bit) != NodeMask{};
}

int resourceNodeFromMap(const TechMap& map, const std::string& port, int pos)
{
    if (map.empty()) {
        return -1;
    }
    for (const auto& expr : map[0]) {
        if (expr.empty() || expr[0].empty()) {
            continue;
        }
        for (const std::string& item : expr[0][0]) {
            size_t first_alpha = item.find_first_not_of("0123456789");
            if (first_alpha == std::string::npos) {
                continue;
            }
            int node = atoi(item.c_str());
            std::string mapped_port = item.substr(first_alpha);
            if (mapped_port == port) {
                return node + pos*256;
            }
        }
    }
    return -1;
}

std::string tileConnKey(const std::string& tile_type, const std::string& wire)
{
    return tile_type + "\n" + wire;
}

std::string tileConnDeltaKey(const std::string& tile_type, const std::string& wire, Coord delta)
{
    return tileConnKey(tile_type, wire) + "\n" + std::to_string(delta.x) + "," + std::to_string(delta.y);
}

void addRouteWireGraphEdge(std::unordered_map<std::string, std::vector<RouteWireGraphEdge>>& graph,
                           const std::string& from_tile_type, const std::string& from_wire,
                           const std::string& to_tile_type, const std::string& to_wire,
                           Coord delta, bool tileconn)
{
    if (from_tile_type.empty() || from_wire.empty() || to_tile_type.empty() || to_wire.empty()) {
        return;
    }
    auto& edges = graph[tileConnKey(from_tile_type, from_wire)];
    auto same = [&](const RouteWireGraphEdge& edge) {
        return edge.tile_type == to_tile_type
            && edge.wire == to_wire
            && edge.delta.x == delta.x
            && edge.delta.y == delta.y
            && edge.tileconn == tileconn;
    };
    if (std::find_if(edges.begin(), edges.end(), same) == edges.end()) {
        edges.push_back(RouteWireGraphEdge{to_tile_type, to_wire, delta, tileconn});
    }
}

void loadTileSiteNames(const std::string& spec_name, std::vector<Referable<Tile>>& tile_grid, const TileGridSpec& grid_spec)
{
    std::ifstream in(spec_name);
    if (!in) {
        return;
    }

    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(in, root)) {
        return;
    }

    for (const std::string& tile_name : root.getMemberNames()) {
        const Json::Value& tile_spec = root[tile_name];
        int grid_x = tile_spec.get("grid_x", -1).asInt();
        int grid_y = tile_spec.get("grid_y", -1).asInt();
        if (grid_x < 0 || grid_y < 0 || grid_x >= grid_spec.size.x || grid_y >= grid_spec.size.y) {
            continue;
        }

        Tile& tile = tile_grid[grid_y*grid_spec.size.x + grid_x];
        tile.full_name = tile_name;
        tile.sites.clear();
        tile.site_types.clear();

        const Json::Value& sites = tile_spec["sites"];
        std::vector<std::string> site_keys = sites.getMemberNames();
        std::sort(site_keys.begin(), site_keys.end());
        for (const std::string& site : site_keys) {
            tile.sites.push_back(site);
            tile.site_types.push_back(sites[site].asString());
        }
    }
}

void assignAttachedCbTiles(std::vector<Referable<Tile>>& tile_grid)
{
    std::unordered_map<std::string, Tile*> physical_cb_by_type_and_vendor_coord;
    for (auto& tile_ref : tile_grid) {
        Tile& tile = tile_ref;
        if (tile.full_name.empty() || !tile.tile_type) {
            continue;
        }
        size_t suffix_pos = tile.full_name.rfind("_X");
        if (suffix_pos == std::string::npos) {
            continue;
        }
        physical_cb_by_type_and_vendor_coord[tile.tile_type->name + tile.full_name.substr(suffix_pos)] = &tile;
    }

    for (auto& tile_ref : tile_grid) {
        Tile& tile = tile_ref;
        tile.cb_coord = Coord{-1, -1};
        tile.cb_full_name.clear();
        if (!tile.cb_type) {
            continue;
        }
        if (!tile.full_name.empty() && tileTypePrefix(tile.full_name) == tile.cb_type->name) {
            tile.cb_coord = tile.coord;
            tile.cb_full_name = tile.full_name;
            continue;
        }
        if (!tile.full_name.empty()) {
            size_t suffix_pos = tile.full_name.rfind("_X");
            if (suffix_pos != std::string::npos) {
                auto it = physical_cb_by_type_and_vendor_coord.find(tile.cb_type->name + tile.full_name.substr(suffix_pos));
                if (it != physical_cb_by_type_and_vendor_coord.end()) {
                    tile.cb_coord = it->second->coord;
                    tile.cb_full_name = it->second->full_name;
                    continue;
                }
            }
        }

        Tile* best = nullptr;
        int best_distance = std::numeric_limits<int>::max();
        for (auto& candidate_ref : tile_grid) {
            Tile& candidate = candidate_ref;
            if (!candidate.tile_type || candidate.tile_type->name != tile.cb_type->name) {
                continue;
            }
            int distance = std::abs(candidate.coord.x - tile.coord.x) + std::abs(candidate.coord.y - tile.coord.y);
            if (!best || distance < best_distance
                || (distance == best_distance && sameVendorTileCoord(tile.full_name, candidate.full_name))) {
                best = &candidate;
                best_distance = distance;
            }
        }
        if (best) {
            tile.cb_coord = best->coord;
            tile.cb_full_name = best->full_name;
        }
    }
}

Tile* attachedRouteTileForCbCoord(std::vector<Referable<Tile>>& tile_grid, const Coord& cb_coord, const CBType* cb_type)
{
    if (!cb_type || cb_coord.x < 0 || cb_coord.y < 0) {
        return nullptr;
    }
    uint16_t expected_base_id = cbBaseTypeId(cb_type);
    static const Referable<Tile>* cached_data = nullptr;
    static size_t cached_size = 0;
    static std::unordered_map<AttachedCbKey, Tile*, AttachedCbKeyHash> cache;
    if (cached_data != tile_grid.data() || cached_size != tile_grid.size()) {
        cached_data = tile_grid.data();
        cached_size = tile_grid.size();
        cache.clear();
        for (auto& tile_ref : tile_grid) {
            Tile& tile = tile_ref;
            if (!tile.cb_type || tile.cb_coord.x < 0 || tile.cb_coord.y < 0) {
                continue;
            }
            AttachedCbKey key{cbBaseTypeId(tile.cb_type), tile.cb_coord.x, tile.cb_coord.y};
            auto it = cache.find(key);
            bool tile_is_physical_cb = tile.coord.x == tile.cb_coord.x && tile.coord.y == tile.cb_coord.y;
            bool old_is_physical_cb = it != cache.end()
                && it->second->coord.x == it->second->cb_coord.x
                && it->second->coord.y == it->second->cb_coord.y;
            if (it == cache.end() || (!old_is_physical_cb && tile_is_physical_cb)) {
                cache[key] = &tile;
            }
        }
    }
    auto it = cache.find(AttachedCbKey{expected_base_id, cb_coord.x, cb_coord.y});
    if (it != cache.end()) {
        return it->second;
    }
    return nullptr;
}

TileJumpTarget resolvedJumpTarget(const Device& device, const Tile& from, int src_node,
                                  Coord delta, uint16_t target_cb_type_id,
                                  NodeMask dst_candidates, bool target_tile_coord,
                                  const std::unordered_map<uint16_t, std::string>* dst_wires,
                                  bool debug)
{
    const CBType* expected_cb_type = cbTypeById(device.cb_types, target_cb_type_id);
    Coord source_cb_coord = from.cb_coord.x >= 0 && from.cb_coord.y >= 0 ? from.cb_coord : from.coord;
    Coord source_coord = target_tile_coord ? from.coord : source_cb_coord;
    Coord next = source_coord + delta;
    auto tile_at = [&](Coord coord) -> Tile* {
        if (coord.x < 0 || coord.y < 0 || coord.x >= device.size_width || coord.y >= device.size_height) {
            return nullptr;
        }
        return const_cast<Referable<Tile>*>(&device.tile_grid[coord.y * device.size_width + coord.x]);
    };
    Tile* next_tile = target_tile_coord
        ? tile_at(next)
        : attachedRouteTileForCbCoord(const_cast<Device&>(device).tile_grid, next, expected_cb_type);
    if (!next_tile || !next_tile->cb_type) {
        if (debug) {
            PNR_LOG("FPGA", "resolveJump reject no target src={} jump={} d=({}, {}) target=({}, {}) target_cb='{}' target_tile_coord={}",
                src_node, src_node, delta.x, delta.y, next.x, next.y,
                expected_cb_type ? expected_cb_type->name : std::string{}, target_tile_coord);
        }
        return {};
    }
    if (!expected_cb_type || cbBaseTypeId(next_tile->cb_type) != cbBaseTypeId(expected_cb_type)) {
        if (debug) {
            PNR_LOG("FPGA", "resolveJump reject target type src={} jump={} target=({}, {}) tile_cb='{}' expected='{}'",
                src_node, src_node, next.x, next.y, next_tile->cb_type->name,
                expected_cb_type ? expected_cb_type->name : std::string{});
        }
        return {};
    }

    next_tile->cb_type->ensureDerivedMasks();
    NodeMask dst_mask = dst_candidates & next_tile->cb_type->valid_dst_nodes;
    if (debug) {
        const std::string* src_name = from.cb_type ? from.cb_type->nodeName(CB_NODE_SRC, src_node) : nullptr;
        PNR_LOG("FPGA", "resolveJump try from=({}, {}) tile='{}' cb='{}' src={} '{}' jump={} d=({}, {}) target=({}, {}) tile='{}' cb='{}' target_tile_coord={} candidates={} valid={} intersect={}",
            from.coord.x, from.coord.y, from.name, from.cb_type ? from.cb_type->name : std::string{}, src_node,
            src_name ? *src_name : std::string{},
            src_node, delta.x, delta.y, next.x, next.y, next_tile->name, next_tile->cb_type->name,
            target_tile_coord, countBits(dst_candidates), countBits(next_tile->cb_type->valid_dst_nodes),
            countBits(dst_mask));
    }
    int dst_node = -1;
    if ((dst_mask & (NodeMask{0,1} << src_node)) != NodeMask{}) {
        dst_node = src_node;
    }
    else {
        dst_mask.for_each_set_bit([&](int bit) {
            dst_node = bit;
            return true;
        });
    }
    if (dst_node < 0) {
        if (debug) {
            PNR_LOG("FPGA", "resolveJump reject no dst src={} jump={} target=({}, {})",
                src_node, src_node, next.x, next.y);
        }
        return {};
    }
    std::string dst_wire;
    if (dst_wires) {
        auto wire_it = dst_wires->find(static_cast<uint16_t>(dst_node));
        if (wire_it != dst_wires->end()) {
            dst_wire = wire_it->second;
        }
    }
    if (dst_wire.empty()) {
        if (const std::string* name = next_tile->cb_type->nodeName(CB_NODE_DST, dst_node)) {
            dst_wire = *name;
        }
    }
    if (debug) {
        PNR_LOG("FPGA", "resolveJump accept src={} jump={} dst={} '{}'",
            src_node, src_node, dst_node, dst_wire);
    }
    return TileJumpTarget{next_tile, dst_node, src_node, dst_wire};
}

void appendResolvedJumpTargets(const Device& device, const Tile& from, int src_node,
                               Coord delta, uint16_t target_cb_type_id,
                               NodeMask dst_candidates, bool target_tile_coord,
                               const std::unordered_map<uint16_t, std::string>* dst_wires,
                               bool debug, std::vector<TileJumpTarget>& targets)
{
    const CBType* expected_cb_type = cbTypeById(device.cb_types, target_cb_type_id);
    Coord source_cb_coord = from.cb_coord.x >= 0 && from.cb_coord.y >= 0 ? from.cb_coord : from.coord;
    Coord source_coord = target_tile_coord ? from.coord : source_cb_coord;
    Coord next = source_coord + delta;
    auto tile_at = [&](Coord coord) -> Tile* {
        if (coord.x < 0 || coord.y < 0 || coord.x >= device.size_width || coord.y >= device.size_height) {
            return nullptr;
        }
        return const_cast<Referable<Tile>*>(&device.tile_grid[coord.y * device.size_width + coord.x]);
    };
    Tile* next_tile = target_tile_coord
        ? tile_at(next)
        : attachedRouteTileForCbCoord(const_cast<Device&>(device).tile_grid, next, expected_cb_type);
    if (!next_tile || !next_tile->cb_type) {
        return;
    }
    if (!expected_cb_type || cbBaseTypeId(next_tile->cb_type) != cbBaseTypeId(expected_cb_type)) {
        return;
    }

    next_tile->cb_type->ensureDerivedMasks();
    NodeMask dst_mask = dst_candidates & next_tile->cb_type->valid_dst_nodes;
    if (debug) {
        const std::string* src_name = from.cb_type ? from.cb_type->nodeName(CB_NODE_SRC, src_node) : nullptr;
        PNR_LOG("FPGA", "resolveJumpTargets from=({}, {}) tile='{}' cb='{}' src={} '{}' d=({}, {}) target=({}, {}) tile='{}' cb='{}' dst_count={}",
            from.coord.x, from.coord.y, from.name, from.cb_type ? from.cb_type->name : std::string{},
            src_node, src_name ? *src_name : std::string{}, delta.x, delta.y,
            next.x, next.y, next_tile->name, next_tile->cb_type->name, countBits(dst_mask));
    }
    dst_mask.for_each_set_bit([&](int dst_node) {
        std::string dst_wire;
        if (dst_wires) {
            auto wire_it = dst_wires->find(static_cast<uint16_t>(dst_node));
            if (wire_it != dst_wires->end()) {
                dst_wire = wire_it->second;
            }
        }
        if (dst_wire.empty()) {
            if (const std::string* name = next_tile->cb_type->nodeName(CB_NODE_DST, dst_node)) {
                dst_wire = *name;
            }
        }
        targets.push_back(TileJumpTarget{next_tile, dst_node, src_node, dst_wire});
        return false;
    });
}

int scaledDirectionAxis(int value, int max_abs)
{
    if (value == 0 || max_abs == 0) {
        return 0;
    }
    int scaled = (std::abs(value) * 7 + max_abs / 2) / max_abs;
    if (scaled == 0) {
        scaled = 1;
    }
    return value < 0 ? -scaled : scaled;
}

Coord jumpTargetBucket(Coord diff)
{
    int abs_x = std::abs(diff.x);
    int abs_y = std::abs(diff.y);
    int max_abs = abs_x > abs_y ? abs_x : abs_y;
    return Coord{scaledDirectionAxis(diff.x, max_abs), scaledDirectionAxis(diff.y, max_abs)};
}

int priorityCrossMagnitude(Coord delta, Coord target)
{
    int cross = delta.x * target.y - delta.y * target.x;
    return cross < 0 ? -cross : cross;
}

bool priorityBehind(Coord delta, Coord target)
{
    if (target.x == 0 && target.y == 0) {
        return false;
    }
    return delta.x * target.x + delta.y * target.y < 0;
}

struct PriorityRank
{
    int behind = 1;
    int cross = 0;
    int length = 0;
    int lane = 0;
    bool valid = false;
};

PriorityRank rankForDelta(Coord delta, Coord target, int src_node)
{
    return PriorityRank{
        priorityBehind(delta, target) ? 1 : 0,
        priorityCrossMagnitude(delta, target),
        std::abs(delta.x) + std::abs(delta.y),
        src_node & 0xf,
        true
    };
}

bool rankBefore(const PriorityRank& lhs, const PriorityRank& rhs)
{
    if (!lhs.valid) {
        return false;
    }
    if (!rhs.valid) {
        return true;
    }
    if (lhs.behind != rhs.behind) {
        return lhs.behind < rhs.behind;
    }
    if (lhs.cross != rhs.cross) {
        return lhs.cross < rhs.cross;
    }
    if (lhs.length != rhs.length) {
        return lhs.length < rhs.length;
    }
    return lhs.lane < rhs.lane;
}

}

namespace fpga {

int testRouteSrcNodeByPhysicalWireName(CBType& cb_type, const std::string& wire)
{
    return routeSrcNodeByPhysicalWireName(cb_type, wire);
}

int testRouteDstNodeByPhysicalWireName(CBType& cb_type, const std::string& wire)
{
    return switchableRouteDstNodeByPhysicalWireName(cb_type, wire);
}

std::string testRouteWireFamilyKey(const std::string& wire)
{
    std::optional<std::string> key = routeWireFamilyKey(wire);
    return key ? *key : std::string{};
}

}

Device& Device::current()
{
    static Device current;
    return current;
}

void Device::loadFromSpec(const std::string& spec_name, const std::string& pins_spec_name)
{
    // tiles specs
    PNR_LOG("FPGA", "loadFromSpec, spec_name: '{}'", spec_name);
    auto phase_start = std::chrono::steady_clock::now();
    auto log_phase = [&](const char* name) {
        auto now = std::chrono::steady_clock::now();
        double seconds = std::chrono::duration<double>(now - phase_start).count();
        PNR_LOG1("FPGA", "loadFromSpec phase '{}' took {:.3f}s", name, seconds);
        phase_start = now;
    };
    std::filesystem::path tileconn = std::filesystem::path(spec_name).parent_path() / "tileconn.json";
    if (std::filesystem::exists(tileconn)) {
        loadTileConnFromSpec(tileconn.string());
    }
    log_phase("tileconn");

    std::map<std::string,TileSpec> tiles_spec;
    readTileGrid(spec_name, &tiles_spec, &grid_spec);
    log_phase("readTileGrid");
    tile_grid.resize(grid_spec.size.y*grid_spec.size.x);

    for (const auto& spec : tiles_spec) {
//            for (const auto& type : tile_types) {
//                if (spec.second.name.find(type.name) == 0) {
                if (std::getenv("SCALEPNR_GRID_POPULATE_LOG")) {
                    PNR_LOG1("FPGA",  "found spec '{}'", spec.second.name);
                }
                for (const auto& rect : spec.second.rects) {
                    Coord name = rect.name;
                    if (std::getenv("SCALEPNR_GRID_POPULATE_LOG")) {
                        PNR_LOG2("FPGA",  "populating rect {}/X{}Y{}...", (Rect)rect, name.x, name.y);
                    }
                    for (int x = rect.x.a; x != rect.x.b+1; ++x) {
                        name.y = rect.name.y;
                        for (int y = rect.y.b; y != rect.y.a-1; --y) {
//                                tile_grid[x*grid_spec.size.y + y].type = std::reference_wrapper(type);
                            tile_grid[y*grid_spec.size.x + x].coord = {x,y};
                            tile_grid[y*grid_spec.size.x + x].name = name;
                            tile_grid[y*grid_spec.size.x + x].cb_type = cbTypeFor(cb_types, local_route_wire_mappings, spec.second.name, x);
                            tile_grid[y*grid_spec.size.x + x].tile_type = tileTypeFor(tile_types, spec.second.name);
                            tile_grid[y*grid_spec.size.x + x].cb = {};
                            tile_grid[y*grid_spec.size.x + x].cb.type = tile_grid[y*grid_spec.size.x + x].cb_type;
                            tile_grid[y*grid_spec.size.x + x].pin_state = {};
                            x_to_grid[name.x] = x;
                            y_to_grid[name.y] = y;
                            ++name.y;
                        }
                        ++name.x;
                    }
                    for (const auto& range : rect.more_x) {
                        name.x = range.name_x;
                        PNR_LOG4("FPGA", " {}/X{}Y{}'", (Range)range, name.x, rect.name.y);
                        for (int x = range.a; x != range.b+1; ++x) {
                            name.y = rect.name.y;
                            for (int y = rect.y.b; y != rect.y.a-1; --y) {
//                                    tile_grid[x*grid_spec.size.y + y].type = std::reference_wrapper(type);
                                tile_grid[y*grid_spec.size.x + x].coord = {x,y};
                                tile_grid[y*grid_spec.size.x + x].name = name;
                                tile_grid[y*grid_spec.size.x + x].cb_type = cbTypeFor(cb_types, local_route_wire_mappings, spec.second.name, x);
                                tile_grid[y*grid_spec.size.x + x].tile_type = tileTypeFor(tile_types, spec.second.name);
                                tile_grid[y*grid_spec.size.x + x].cb = {};
                                tile_grid[y*grid_spec.size.x + x].cb.type = tile_grid[y*grid_spec.size.x + x].cb_type;
                                tile_grid[y*grid_spec.size.x + x].pin_state = {};
                                x_to_grid[name.x] = x;
                                y_to_grid[name.y] = y;
                                ++name.y;
                            }
                            ++name.x;
                        }
                    }
                }
//                }
//            }
    }
    log_phase("populateGrid");

    size_width = grid_spec.size.x;
    size_height = grid_spec.size.y;
    loadTileSiteNames(spec_name, tile_grid, grid_spec);
    log_phase("loadTileSiteNames");
    assignAttachedCbTiles(tile_grid);
    log_phase("assignAttachedCbTiles");
    applyTileConnSubtypes();
    log_phase("applyTileConnSubtypes");
    cnt_regs = 2*grid_spec.size.y*grid_spec.size.x*4;
    cnt_luts = 2*grid_spec.size.y*grid_spec.size.x*4;
    PNR_LOG("FPGA", "loadFromSpec, fpga width: {}, height: {}, cnt_regs: {}, cnt_luts: {}, pins_spec_name: '{}'", size_width, size_height, cnt_regs, cnt_luts, pins_spec_name);

    PNR_LOG("FPGA", "loadFromSpec pins, pins_spec_name: '{}'", pins_spec_name);
    std::vector<PinSpec> specs;
    readPackagePins(pins_spec_name, specs);

    for (auto spec : specs) {
        pins.push_back(Pin{spec.name, spec.bank, spec.site, spec.tile, spec.function, spec.pos});
    }

}

void Device::applyTileConnSubtypes()
{
    if (tile_grid.empty() || tileconn_rules.empty()) {
        return;
    }

    std::vector<uint16_t> initial_tile_type_ids(tile_grid.size(), CB_INVALID_TYPE_ID);
    for (size_t index = 0; index < tile_grid.size(); ++index) {
        if (tile_grid[index].cb_type) {
            initial_tile_type_ids[index] = tile_grid[index].cb_type->type_id;
        }
    }
    cb_types.reserve(cb_types.size() + 256);
    for (size_t index = 0; index < tile_grid.size(); ++index) {
        uint16_t type_id = initial_tile_type_ids[index];
        if (type_id == CB_INVALID_TYPE_ID || type_id >= cb_types.size()) {
            continue;
        }
        tile_grid[index].cb_type = &cb_types[type_id];
        tile_grid[index].cb.type = tile_grid[index].cb_type;
    }

    std::unordered_map<std::string, uint16_t> cb_by_name;
    cb_by_name.reserve(cb_types.size());
    for (CBType& cb_type : cb_types) {
        cb_by_name.emplace(cb_type.name, cb_type.type_id);
    }
    std::unordered_map<std::string, std::vector<const ParsedTileConnRule*>> rules_from_type;
    std::unordered_map<std::string, std::vector<const ParsedTileConnRule*>> rules_to_type;
    rules_from_type.reserve(tileconn_rules.size());
    rules_to_type.reserve(tileconn_rules.size());
    for (const ParsedTileConnRule& rule : tileconn_rules) {
        rules_from_type[rule.from_tile_type].push_back(&rule);
        rules_to_type[rule.to_tile_type].push_back(&rule);
    }

    for (CBType& cb_type : cb_types) {
        cb_type.ensureDerivedMasks();
    }

    std::vector<uint16_t> base_type_ids(tile_grid.size(), CB_INVALID_TYPE_ID);
    std::vector<uint16_t> assigned_type_ids(tile_grid.size(), CB_INVALID_TYPE_ID);
    std::vector<std::vector<uint16_t>> variants_by_base(cb_types.size());
    for (uint16_t id = 0; id < cb_types.size(); ++id) {
        variants_by_base[id].push_back(id);
    }
    std::vector<NodeMask> active_src_mask_by_base(cb_types.size());
    for (uint16_t id = 0; id < cb_types.size(); ++id) {
        uint16_t base_id = cbBaseTypeId(&cb_types[id]);
        if (base_id >= active_src_mask_by_base.size()) {
            continue;
        }
        // dst_by_src is produced by this pass, so source enumeration must come
        // from structural route masks instead of already-resolved mappings.
        active_src_mask_by_base[base_id] |= jumpSourcesUsedByRouteMasks(cb_types[id]);
        for (int src_node = 0; src_node < CB_MAX_NODES; ++src_node) {
            if (cb_types[id].dst_by_src[src_node].empty()) {
                continue;
            }
            active_src_mask_by_base[base_id] |= NodeMask{0,1} << src_node;
        }
    }
    std::vector<std::vector<uint16_t>> active_srcs_by_base(cb_types.size());
    for (uint16_t base_id = 0; base_id < active_src_mask_by_base.size(); ++base_id) {
        const CBType& base = cb_types[base_id];
        active_src_mask_by_base[base_id].for_each_set_bit([&](int src_node) {
            if (!base.nodeName(CB_NODE_SRC, src_node)
                && base.dst_by_src[src_node].empty()) {
                return false;
            }
            active_srcs_by_base[base_id].push_back(static_cast<uint16_t>(src_node));
            return false;
        });
    }
    auto compute_source_wires_for_src = [&](const CBType& base, uint16_t src_node) {
        std::vector<std::string> wires;
        for (const auto& [wire, node] : base.src_nodes_by_name) {
            if (node != src_node) {
                continue;
            }
            auto edges_it = route_wire_graph.find(tileConnKey(base.name, wire));
            if (edges_it == route_wire_graph.end()) {
                continue;
            }
            if (std::any_of(edges_it->second.begin(), edges_it->second.end(),
                    [](const RouteWireGraphEdge& edge) { return edge.tileconn; })) {
                wires.push_back(wire);
            }
        }
        if (wires.empty()) {
            if (const std::string* src_wire = base.nodeName(CB_NODE_SRC, src_node)) {
                auto edges_it = route_wire_graph.find(tileConnKey(base.name, *src_wire));
                if (edges_it != route_wire_graph.end()
                    && std::any_of(edges_it->second.begin(), edges_it->second.end(),
                        [](const RouteWireGraphEdge& edge) { return edge.tileconn; })) {
                    wires.push_back(*src_wire);
                }
            }
        }
        std::sort(wires.begin(), wires.end());
        wires.erase(std::unique(wires.begin(), wires.end()), wires.end());
        return wires;
    };
    std::vector<std::unordered_map<uint16_t, std::vector<std::string>>> source_wires_by_base(cb_types.size());
    std::vector<std::vector<uint16_t>> tileconn_srcs_by_base(cb_types.size());
    for (uint16_t base_id = 0; base_id < active_srcs_by_base.size(); ++base_id) {
        const CBType& base = cb_types[base_id];
        for (uint16_t src_node : active_srcs_by_base[base_id]) {
            std::vector<std::string> wires = compute_source_wires_for_src(base, src_node);
            if (!wires.empty()) {
                tileconn_srcs_by_base[base_id].push_back(src_node);
                source_wires_by_base[base_id].emplace(src_node, std::move(wires));
            }
        }
    }
    auto source_wires_for_src = [&](uint16_t base_id, uint16_t src_node) -> const std::vector<std::string>& {
        static const std::vector<std::string> empty;
        if (base_id >= source_wires_by_base.size()) {
            return empty;
        }
        auto it = source_wires_by_base[base_id].find(src_node);
        return it == source_wires_by_base[base_id].end() ? empty : it->second;
    };

    struct SyntheticNodeKey
    {
        uint16_t base_id = CB_INVALID_TYPE_ID;
        CBNodeNameType role = CB_NODE_JUMP;
        std::string tile_type;
        std::string wire;
        Coord delta;

        bool operator<(const SyntheticNodeKey& other) const
        {
            return std::tie(base_id, role, tile_type, wire, delta.x, delta.y)
                < std::tie(other.base_id, other.role, other.tile_type, other.wire, other.delta.x, other.delta.y);
        }
    };

    std::map<SyntheticNodeKey, int> synthetic_node_by_key;
    std::vector<std::array<NodeMask, 2>> synthetic_used_by_base(cb_types.size());
    for (uint16_t id = 0; id < cb_types.size(); ++id) {
        synthetic_used_by_base[id][0] = jumpNodesStructurallyUsedAs(cb_types[id], CB_NODE_SRC);
        synthetic_used_by_base[id][1] = jumpNodesStructurallyUsedAs(cb_types[id], CB_NODE_DST);
    }

    auto role_index = [](CBNodeNameType type) {
        return type == CB_NODE_SRC ? 0 : 1;
    };

    auto synthetic_node_name = [](const std::string& tile_type, const std::string& wire, Coord delta) {
        return tile_type + "." + wire + "@"
            + std::to_string(delta.x) + "," + std::to_string(delta.y);
    };

    auto remember_synthetic_node = [&](CBType& cb_type, CBNodeNameType role,
                                       const std::string& tile_type, const std::string& wire,
                                       Coord delta, int node) {
        std::string name = synthetic_node_name(tile_type, wire, delta);
        cb_type.rememberNodeName(role, node, name);
        if (role == CB_NODE_SRC) {
            cb_type.rememberNodeName(CB_NODE_JUMP, node, name);
        }
    };

    auto reserve_synthetic_node = [&](uint16_t base_id, CBNodeNameType role, const std::string& tile_type,
                                      const std::string& wire, Coord delta,
                                      const std::string& path) {
        PNR_ASSERT(base_id != CB_INVALID_TYPE_ID && base_id < cb_types.size(),
            "synthetic route node requested for invalid base type {}", base_id);
        PNR_ASSERT(role == CB_NODE_SRC || role == CB_NODE_DST,
            "synthetic route node role must be SRC or DST, got {}", static_cast<int>(role));
        if (!signed4DeltaInRange(delta)) {
            return -1;
        }
        if (base_id >= synthetic_used_by_base.size()) {
            synthetic_used_by_base.resize(base_id + 1);
            synthetic_used_by_base[base_id][0] = jumpNodesStructurallyUsedAs(cb_types[base_id], CB_NODE_SRC);
            synthetic_used_by_base[base_id][1] = jumpNodesStructurallyUsedAs(cb_types[base_id], CB_NODE_DST);
        }
        auto role_used = [&](CBNodeNameType type, int node) {
            return (synthetic_used_by_base[base_id][role_index(type)] & (NodeMask{0,1} << node)) != NodeMask{};
        };
        auto mark_role_used = [&](CBNodeNameType type, int node) {
            synthetic_used_by_base[base_id][role_index(type)] |= NodeMask{0,1} << node;
        };
        SyntheticNodeKey key{base_id, role, tile_type, wire, delta};
        if (auto it = synthetic_node_by_key.find(key); it != synthetic_node_by_key.end()) {
            return it->second;
        }
        CBType& base = cb_types[base_id];
        CBNodeNameType other_role = role == CB_NODE_SRC ? CB_NODE_DST : CB_NODE_SRC;
        SyntheticNodeKey other_key{base_id, other_role, tile_type, wire, delta};
        if (auto other_it = synthetic_node_by_key.find(other_key); other_it != synthetic_node_by_key.end()) {
            int node = other_it->second;
            if (!role_used(role, node)) {
                synthetic_node_by_key.emplace(key, node);
                mark_role_used(role, node);
                return node;
            }
        }
        int existing_node = nodeNumByPhysicalWireNameImpl(base, role, wire, false);
        if (existing_node >= 0) {
            bool matching_encoded_delta = role != CB_NODE_SRC;
            if (role == CB_NODE_SRC) {
                Coord encoded_delta = jumpDeltaFromNodeLocal(existing_node);
                matching_encoded_delta = encoded_delta.x == delta.x && encoded_delta.y == delta.y;
            }
            if (matching_encoded_delta) {
                synthetic_node_by_key.emplace(key, existing_node);
                mark_role_used(role, existing_node);
                return existing_node;
            }
        }
        auto edge_it = route_wire_graph.find(tileConnKey(tile_type, wire));
        if (edge_it != route_wire_graph.end()) {
            for (const RouteWireGraphEdge& edge : edge_it->second) {
                if (!exactCBTypeFor(cb_types, edge.tile_type)) {
                    continue;
                }
                if ((edge.delta.x != delta.x || edge.delta.y != delta.y)
                    && (edge.delta.x != -delta.x || edge.delta.y != -delta.y)) {
                    continue;
                }
                int node = nodeNumByPhysicalWireName(base, role, edge.wire);
                if (node >= 0) {
                    bool matching_encoded_delta = role != CB_NODE_SRC;
                    if (role == CB_NODE_SRC) {
                        Coord encoded_delta = jumpDeltaFromNodeLocal(node);
                        matching_encoded_delta = encoded_delta.x == delta.x && encoded_delta.y == delta.y;
                    }
                    if (matching_encoded_delta) {
                        synthetic_node_by_key.emplace(key, node);
                        mark_role_used(role, node);
                        return node;
                    }
                }
            }
        }
        auto reserve_in_slot_delta = [&](Coord slot_delta) {
            if (!signed4DeltaInRange(slot_delta)) {
                return -1;
            }
            for (int num = 0; num < 16; ++num) {
                int node = jumpIndexForDeltaLocal(slot_delta, num);
                if (role_used(role, node)) {
                    continue;
                }
                synthetic_node_by_key.emplace(key, node);
                mark_role_used(role, node);
                return node;
            }
            return -1;
        };

        if (int node = reserve_in_slot_delta(delta); node >= 0) {
            return node;
        }

        std::string lanes;
        for (int num = 0; num < 16; ++num) {
            int node = jumpIndexForDeltaLocal(delta, num);
            if (!lanes.empty()) {
                lanes += "; ";
            }
            lanes += "num=" + std::to_string(num) + " node=" + std::to_string(node);
            if (role == CB_NODE_SRC) {
                const std::string* src_name = base.nodeName(CB_NODE_SRC, node);
                const std::string* jump_name = base.nodeName(CB_NODE_JUMP, node);
                if (src_name) {
                    lanes += " src='" + *src_name + "'";
                }
                if (jump_name) {
                    lanes += " jump='" + *jump_name + "'";
                }
            }
            if (role == CB_NODE_DST) {
                const std::string* dst_name = base.nodeName(CB_NODE_DST, node);
                if (dst_name) {
                    lanes += " dst='" + *dst_name + "'";
                }
            }
            bool occupied_for_role = role_used(role, node);
            lanes += occupied_for_role ? " occupied_for_role" : " free_for_role";
        }
        PNR_ASSERT(false,
            "pass-through jump .num overuse: cb='{}' base_id={} role={} tile_type='{}' wire='{}' delta=({}, {}) lanes=[{}] path={}",
            base.name, base_id, static_cast<int>(role), tile_type, wire, delta.x, delta.y, lanes, path);
        return -1;
    };

    auto materialize_synthetic_node = [&](CBType& cb_type, uint16_t base_id,
                                          CBNodeNameType role,
                                          const std::string& tile_type,
                                          const std::string& wire, Coord delta,
                                          const std::string& path) {
        int node = reserve_synthetic_node(base_id, role, tile_type, wire, delta, path);
        remember_synthetic_node(cb_type, role, tile_type, wire, delta, node);
        return node;
    };

    auto copy_source_incoming_arcs = [&](CBType& cb_type, int old_src, int new_src) {
        if (old_src < 0 || old_src >= CB_MAX_NODES || new_src < 0 || new_src >= CB_MAX_NODES
            || old_src == new_src) {
            return false;
        }
        NodeMask old_bit = NodeMask{0,1} << old_src;
        NodeMask new_bit = NodeMask{0,1} << new_src;
        bool changed_masks = false;
        auto copy_jump_arc = [&](auto& table) {
            for (auto& [node, state] : table.values) {
                (void)node;
                if ((state.jump & old_bit) == NodeMask{} || (state.jump & new_bit) != NodeMask{}) {
                    continue;
                }
                state.jump |= new_bit;
                changed_masks = true;
            }
        };
        copy_jump_arc(cb_type.local_src);
        copy_jump_arc(cb_type.dst_src);
        copy_jump_arc(cb_type.joint_src);
        if (cb_type.src_joint[old_src].joint != NodeMask{}) {
            cb_type.src_joint[new_src].joint |= cb_type.src_joint[old_src].joint;
            changed_masks = true;
        }
        if (changed_masks) {
            cb_type.derived_masks_valid = false;
        }
        return changed_masks;
    };

    auto source_node_for_resolved_delta = [&](CBType& cb_type, uint16_t base_id,
                                              const CBType& source_base,
                                              int src_node, const std::string& src_wire,
                                              Coord final_delta, const std::string& path) {
        (void)cb_type;
        (void)base_id;
        (void)source_base;
        (void)src_wire;
        (void)final_delta;
        (void)path;
        return src_node;
    };

    auto add_mapping = [&](CBType& cb_type, std::array<bool, CB_MAX_NODES>& overridden_src,
                           int src_node, const CBType& target_cb_type, int dst_node, Coord delta,
                           const Coord& source_coord, const std::string& src_wire, const std::string& dst_wire,
                           bool target_tile_coord = false) {
        bool debug = debugTileConnCoord(source_coord) || debugTileConnWire(src_wire) || debugTileConnWire(dst_wire);
        if (debug) {
            PNR_LOG("FPGA",
                "tileconn map coord=({}, {}) source_cb='{}' src_wire='{}' src_node={} target_cb='{}' dst_wire='{}' dst_node={} delta=({}, {}) target_tile_coord={}",
                source_coord.x, source_coord.y, cb_type.name, src_wire, src_node,
                target_cb_type.name, dst_wire, dst_node, delta.x, delta.y, target_tile_coord);
        }
        if (src_node < 0 || src_node >= CB_MAX_NODES || dst_node < 0 || dst_node >= CB_MAX_NODES) {
            if (debug) {
                PNR_LOG("FPGA",
                    "tileconn map skip coord=({}, {}) source_cb='{}' src_wire='{}' src_node={} target_cb='{}' dst_wire='{}' dst_node={}",
                    source_coord.x, source_coord.y, cb_type.name, src_wire, src_node,
                    target_cb_type.name, dst_wire, dst_node);
            }
            return;
        }
        if (!overridden_src[src_node]) {
            cb_type.dst_by_src[src_node].clear();
            overridden_src[src_node] = true;
        }
        addResolvedJumpBit(cb_type.dst_by_src[src_node], delta,
                           target_cb_type.type_id, dst_node,
                           cb_type.name, src_node, target_cb_type.name,
                           dst_wire,
                           target_tile_coord);
        addSrcPriorityDelta(cb_type, src_node, delta);
        cb_type.derived_masks_valid = false;
    };

    struct IncidentRouteEdge
    {
        Coord delta;
        std::string tile_type;
        std::string wire;
    };

    size_t signature_cache_hits = 0;
    size_t signature_cache_misses = 0;
    size_t target_dst_calls = 0;
    size_t target_dst_cache_hits = 0;
    size_t target_dst_success = 0;
    size_t target_dst_failed = 0;
    size_t target_dst_bfs_pops = 0;
    size_t valid_edges_calls = 0;
    size_t valid_edges_returned = 0;

    auto tile_at_coord = [&](Coord coord) -> Tile* {
        if (coord.x < 0 || coord.y < 0 || coord.x >= size_width || coord.y >= size_height) {
            return nullptr;
        }
        return &tile_grid[coord.y * size_width + coord.x];
    };

    auto valid_edges_from_tile_wire = [&](const std::string& tile_type, const std::string& wire,
                                          Coord coord, bool tileconn_only = false) {
        ++valid_edges_calls;
        std::vector<IncidentRouteEdge> edges;
        auto edge_it = route_wire_graph.find(tileConnKey(tile_type, wire));
        if (edge_it == route_wire_graph.end()) {
            return edges;
        }
        for (const RouteWireGraphEdge& edge : edge_it->second) {
            if (tileconn_only && !edge.tileconn) {
                continue;
            }
            if (edge.delta.x == 0 && edge.delta.y == 0) {
                continue;
            }
            Coord next_coord{coord.x + edge.delta.x, coord.y + edge.delta.y};
            Tile* next_tile = tile_at_coord(next_coord);
            if (!next_tile || !next_tile->tile_type || next_tile->tile_type->name != edge.tile_type) {
                continue;
            }
            edges.push_back(IncidentRouteEdge{edge.delta, edge.tile_type, edge.wire});
        }
        valid_edges_returned += edges.size();
        return edges;
    };

    std::map<std::string, std::vector<std::string>> route_wires_by_type;
    for (const auto& [key, edges] : route_wire_graph) {
        size_t sep = key.find('\n');
        if (sep == std::string::npos) {
            continue;
        }
        const std::string tile_type = key.substr(0, sep);
        const std::string wire = key.substr(sep + 1);
        if (std::any_of(edges.begin(), edges.end(), [](const RouteWireGraphEdge& edge) {
                return edge.delta.x != 0 || edge.delta.y != 0;
            })) {
            route_wires_by_type[tile_type].push_back(wire);
        }
    }
    for (auto& [type, wires] : route_wires_by_type) {
        std::sort(wires.begin(), wires.end());
        wires.erase(std::unique(wires.begin(), wires.end()), wires.end());
    }

    auto route_path_node = [](const std::string& tile_type, Coord coord, const std::string& wire) {
        return tile_type + "@(" + std::to_string(coord.x) + "," + std::to_string(coord.y) + ")." + wire;
    };

    struct InternalWirePath
    {
        std::string wire;
        std::string path;
    };

    auto internal_reachable_wires = [&](const std::string& tile_type, Coord coord, const std::string& start_wire) {
        std::vector<InternalWirePath> wires;
        std::deque<InternalWirePath> queue;
        std::unordered_set<std::string> seen;
        queue.push_back(InternalWirePath{start_wire, route_path_node(tile_type, coord, start_wire)});
        seen.insert(start_wire);
        constexpr int max_internal_wire_steps = 12;
        int depth = 0;
        while (!queue.empty() && depth <= max_internal_wire_steps) {
            size_t level = queue.size();
            while (level-- != 0) {
                InternalWirePath item = std::move(queue.front());
                queue.pop_front();
                wires.push_back(item);
                auto edge_it = route_wire_graph.find(tileConnKey(tile_type, item.wire));
                if (edge_it == route_wire_graph.end()) {
                    continue;
                }
                for (const RouteWireGraphEdge& edge : edge_it->second) {
                    if (edge.tile_type != tile_type || edge.delta.x != 0 || edge.delta.y != 0) {
                        continue;
                    }
                    if (seen.insert(edge.wire).second) {
                        queue.push_back(InternalWirePath{
                            edge.wire,
                            item.path + " -> " + route_path_node(edge.tile_type, coord, edge.wire),
                        });
                    }
                }
            }
            ++depth;
        }
        std::sort(wires.begin(), wires.end(), [](const InternalWirePath& lhs, const InternalWirePath& rhs) {
            return lhs.wire < rhs.wire;
        });
        wires.erase(std::unique(wires.begin(), wires.end(), [](const InternalWirePath& lhs, const InternalWirePath& rhs) {
            return lhs.wire == rhs.wire;
        }), wires.end());
        return wires;
    };

    struct PendingRule
    {
        const ParsedTileConnRule* rule = nullptr;
        uint16_t target_base_id = CB_INVALID_TYPE_ID;
        bool reverse = false;
    };

    struct TargetDstCacheEntry
    {
        bool resolved = false;
        uint16_t target_base_id = CB_INVALID_TYPE_ID;
        int dst_node = -1;
        Coord target_coord{};
        std::string target_wire;
    };

    std::unordered_map<std::string, TargetDstCacheEntry> target_dst_cache;
    auto target_dst_cache_key = [](const std::string& current_tile_type, const std::string& current_wire,
                                   Coord current_coord,
                                   const IncidentRouteEdge& edge) {
        return current_tile_type + "\n"
            + current_wire + "\n"
            + std::to_string(current_coord.x) + "," + std::to_string(current_coord.y) + "\n"
            + std::to_string(edge.delta.x) + "," + std::to_string(edge.delta.y) + "\n"
            + edge.tile_type + "\n" + edge.wire;
    };

    for (size_t index = 0; index < tile_grid.size(); ++index) {
        Tile& tile = tile_grid[index];
        if (!tile.cb_type) {
            continue;
        }
        base_type_ids[index] = tile.cb_type->type_id;
    }

    size_t specialized_tiles = 0;
    size_t created_subtypes = 0;
    std::map<std::vector<uint64_t>, uint16_t> subtype_by_topology_signature;
    for (size_t index = 0; index < tile_grid.size(); ++index) {
        if (std::getenv("SCALEPNR_TILECONN_PROGRESS") && (index % 1000) == 0) {
            PNR_LOG("FPGA", "applyTileConnSubtypes progress index={}/{} created={} specialized={} sig_hit={} sig_miss={} target_calls={} target_hit={} target_ok={} target_fail={} bfs_pops={} edge_calls={} edge_returned={}",
                index, tile_grid.size(), created_subtypes, specialized_tiles,
                signature_cache_hits, signature_cache_misses,
                target_dst_calls, target_dst_cache_hits, target_dst_success, target_dst_failed,
                target_dst_bfs_pops, valid_edges_calls, valid_edges_returned);
        }
        Tile& tile = tile_grid[index];
        uint16_t base_id = base_type_ids[index];
        if (base_id == CB_INVALID_TYPE_ID || base_id >= cb_types.size()) {
            continue;
        }
        const CBType& base = cb_types[base_id];
        std::vector<PendingRule> pending_rules;
        Coord source_cb_coord = tile.cb_coord.x >= 0 && tile.cb_coord.y >= 0 ? tile.cb_coord : tile.coord;
        if (tile.coord.x != source_cb_coord.x || tile.coord.y != source_cb_coord.y) {
            continue;
        }
        if (debugTileConnCoord(tile.coord)) {
            PNR_LOG("FPGA", "applyTileConnSubtypes tile index={} coord=({}, {}) cb='{}' sources={}",
                index, tile.coord.x, tile.coord.y, base.name, tileconn_srcs_by_base[base_id].size());
        }

        if (auto rules_it = rules_from_type.find(base.name); rules_it != rules_from_type.end()) {
            for (const ParsedTileConnRule* rule_ptr : rules_it->second) {
                const ParsedTileConnRule& rule = *rule_ptr;
                Coord target_coord = source_cb_coord + rule.delta;
                uint16_t target_base_id = CB_INVALID_TYPE_ID;
                const CBType* expected_target = nullptr;
                if (auto cb_it = cb_by_name.find(rule.to_tile_type); cb_it != cb_by_name.end()) {
                    expected_target = cbTypeById(cb_types, cb_it->second);
                }
                Tile* target = attachedRouteTileForCbCoord(tile_grid, target_coord, expected_target);
                if (target && target->cb_type) {
                    target_base_id = cbBaseTypeId(target->cb_type);
                    if (target_base_id != CB_INVALID_TYPE_ID
                        && target_base_id < cb_types.size()
                        && cb_types[target_base_id].name == rule.to_tile_type) {
                        pending_rules.push_back(PendingRule{&rule, target_base_id, false});
                    }
                    else {
                        target_base_id = CB_INVALID_TYPE_ID;
                    }
                }
            }
        }
        if (auto rules_it = rules_to_type.find(base.name); rules_it != rules_to_type.end()) {
            for (const ParsedTileConnRule* rule_ptr : rules_it->second) {
                const ParsedTileConnRule& rule = *rule_ptr;
                Coord target_coord = source_cb_coord - rule.delta;
                uint16_t target_base_id = CB_INVALID_TYPE_ID;
                const CBType* expected_target = nullptr;
                if (auto cb_it = cb_by_name.find(rule.from_tile_type); cb_it != cb_by_name.end()) {
                    expected_target = cbTypeById(cb_types, cb_it->second);
                }
                Tile* target = attachedRouteTileForCbCoord(tile_grid, target_coord, expected_target);
                if (target && target->cb_type) {
                    target_base_id = cbBaseTypeId(target->cb_type);
                    if (target_base_id != CB_INVALID_TYPE_ID
                        && target_base_id < cb_types.size()
                        && cb_types[target_base_id].name == rule.from_tile_type) {
                        pending_rules.push_back(PendingRule{&rule, target_base_id, true});
                    }
                    else {
                        target_base_id = CB_INVALID_TYPE_ID;
                    }
                }
            }
        }

        auto non_cb_edge_continues = [&](const std::string& current_tile_type,
                                         const IncidentRouteEdge& edge, Coord current_coord) {
            Coord next_coord{current_coord.x + edge.delta.x, current_coord.y + edge.delta.y};
            Tile* next_tile = tile_at_coord(next_coord);
            if (!next_tile || !next_tile->tile_type
                || (next_tile->tile_type->sites.empty() && next_tile->tile_type->elements.empty())) {
                return false;
            }
            for (const IncidentRouteEdge& next_edge : valid_edges_from_tile_wire(edge.tile_type, edge.wire, next_coord, true)) {
                if (next_edge.tile_type == current_tile_type
                    && next_edge.delta.x == -edge.delta.x
                    && next_edge.delta.y == -edge.delta.y) {
                    continue;
                }
                return true;
            }
            return false;
        };

        auto target_dst_for_edge = [&](const std::string& current_tile_type, const std::string& current_wire,
                                       const IncidentRouteEdge& edge, Coord current_coord,
                                       uint16_t& target_base_id, int& dst_node,
                                       Coord& target_coord_out, std::string& target_wire_out,
                                       const std::string& path) {
            (void)path;
            ++target_dst_calls;
            std::string cache_key = target_dst_cache_key(current_tile_type, current_wire, current_coord, edge);
            if (auto cached = target_dst_cache.find(cache_key); cached != target_dst_cache.end()) {
                ++target_dst_cache_hits;
                if (!cached->second.resolved) {
                    return false;
                }
                target_base_id = cached->second.target_base_id;
                dst_node = cached->second.dst_node;
                target_coord_out = cached->second.target_coord;
                target_wire_out = cached->second.target_wire;
                return true;
            }

            auto remember_failed = [&]() {
                target_dst_cache.emplace(cache_key, TargetDstCacheEntry{});
                return false;
            };
            auto remember_success = [&]() {
                target_dst_cache.emplace(cache_key, TargetDstCacheEntry{
                    true, target_base_id, dst_node, target_coord_out, target_wire_out});
                return true;
            };

            Coord next_coord{current_coord.x + edge.delta.x, current_coord.y + edge.delta.y};
            Tile* next_tile = tile_at_coord(next_coord);
            if (!next_tile || !next_tile->tile_type || next_tile->tile_type->name != edge.tile_type) {
                return remember_failed();
            }
            std::optional<std::string> source_family = routeWireFamilyKey(current_wire);

            struct QueueItem
            {
                Coord coord;
                std::string tile_type;
                std::string wire;
                int depth = 0;
            };
            std::deque<QueueItem> queue;
            std::unordered_set<std::string> seen;
            auto seen_key = [](Coord coord, const std::string& tile_type, const std::string& wire) {
                return std::to_string(coord.x) + "," + std::to_string(coord.y) + "\n"
                    + tile_type + "\n" + wire;
            };
            seen.insert(seen_key(current_coord, current_tile_type, current_wire));
            seen.insert(seen_key(next_coord, edge.tile_type, edge.wire));
            queue.push_back(QueueItem{next_coord, edge.tile_type, edge.wire, 0});

            constexpr int max_route_wire_depth = 64;
            while (!queue.empty()) {
                QueueItem item = std::move(queue.front());
                queue.pop_front();
                ++target_dst_bfs_pops;

                Tile* item_tile = tile_at_coord(item.coord);
                if (!item_tile || !item_tile->tile_type || item_tile->tile_type->name != item.tile_type) {
                    continue;
                }

                if (CBType* exact_target = exactCBTypeFor(cb_types, item.tile_type)) {
                    if (cbHasRouteGraph(*exact_target)) {
                        if (!sameRouteWireFamily(source_family, item.wire)) {
                            continue;
                        }
                        int exact_dst = switchableRouteDstNodeByPhysicalWireName(*exact_target, item.wire);
                        if (exact_dst >= 0) {
                            target_base_id = cbBaseTypeId(exact_target);
                            dst_node = exact_dst;
                            target_coord_out = item.coord;
                            target_wire_out = item.wire;
                            if (target_base_id != CB_INVALID_TYPE_ID && target_base_id < cb_types.size()) {
                                ++target_dst_success;
                                return remember_success();
                            }
                            ++target_dst_failed;
                            return remember_failed();
                        }
                    }
                }

                if (item.depth >= max_route_wire_depth) {
                    continue;
                }
                for (const IncidentRouteEdge& next_edge : valid_edges_from_tile_wire(item.tile_type, item.wire, item.coord, true)) {
                    if (!sameRouteWireFamily(source_family, next_edge.wire)) {
                        continue;
                    }
                    Coord follow_coord{item.coord.x + next_edge.delta.x, item.coord.y + next_edge.delta.y};
                    Tile* follow_tile = tile_at_coord(follow_coord);
                    if (!follow_tile || !follow_tile->tile_type || follow_tile->tile_type->name != next_edge.tile_type) {
                        continue;
                    }
                    std::string key = seen_key(follow_coord, next_edge.tile_type, next_edge.wire);
                    if (!seen.insert(key).second) {
                        continue;
                    }
                    queue.push_back(QueueItem{follow_coord, next_edge.tile_type, next_edge.wire, item.depth + 1});
                }
            }
            ++target_dst_failed;
            return remember_failed();
        };

        std::vector<uint64_t> signature;
        signature.reserve(8 + 9 * 3 + pending_rules.size() * 3);
        signature.push_back(base_id);
        signature.push_back(tile.tile_type ? static_cast<uint64_t>(tile.tile_type->num) : 0);
        for (int dy = -8; dy <= 8; ++dy) {
            for (int dx = -8; dx <= 8; ++dx) {
                Tile* nearby = tile_at_coord(Coord{source_cb_coord.x + dx, source_cb_coord.y + dy});
                signature.push_back(nearby && nearby->tile_type
                    ? static_cast<uint64_t>(nearby->tile_type->num)
                    : std::numeric_limits<uint64_t>::max());
            }
        }
        signature.push_back(CB_INVALID_TYPE_ID);
        for (const PendingRule& pending : pending_rules) {
            signature.push_back(static_cast<uint64_t>(reinterpret_cast<uintptr_t>(pending.rule)));
            signature.push_back(pending.target_base_id);
            signature.push_back(pending.reverse ? 1 : 0);
        }
        bool debug_this_coord = debugTileConnCoord(tile.coord) || debugTileConnCoord(source_cb_coord);
        if (auto cached = subtype_by_topology_signature.find(signature);
            cached != subtype_by_topology_signature.end() && !debug_this_coord) {
            ++signature_cache_hits;
            assigned_type_ids[index] = cached->second;
            if (cached->second != base_id) {
                ++specialized_tiles;
            }
            continue;
        }
        ++signature_cache_misses;

        auto candidate_storage = std::make_unique<CBType>(base);
        CBType& candidate = *candidate_storage;
        std::array<bool, CB_MAX_NODES> overridden_src{};
        bool changed = false;

        // Tileconn subtyping owns jump landing resolution for active route
        // sources; do not keep stale generic dst_by_src entries in variants.
        for (uint16_t src_node : tileconn_srcs_by_base[base_id]) {
            if (!candidate.dst_by_src[src_node].empty()) {
                candidate.dst_by_src[src_node].clear();
                candidate.derived_masks_valid = false;
                changed = true;
            }
            overridden_src[src_node] = true;
        }

        auto apply_rule_side_resolved = [&](const CBType& source_base, const CBType& target_base,
                                            const ParsedTileConnRule& rule, bool reverse) {
            Coord delta = reverse ? Coord{-rule.delta.x, -rule.delta.y} : rule.delta;
            for (const ParsedTileConnPair& pair : rule.wire_pairs) {
                const std::string& src_wire = reverse ? pair.to_wire : pair.from_wire;
                const std::string& dst_wire = reverse ? pair.from_wire : pair.to_wire;
                int src_node = nodeNumByPhysicalWireNameImpl(source_base, CB_NODE_SRC, src_wire, false);
                std::string path = route_path_node(source_base.name, source_cb_coord, src_wire)
                    + " -> " + route_path_node(target_base.name,
                        Coord{source_cb_coord.x + delta.x, source_cb_coord.y + delta.y}, dst_wire);
                uint16_t final_target_base_id = CB_INVALID_TYPE_ID;
                int dst_node = -1;
                Coord final_target_coord{};
                std::string final_target_wire;
                IncidentRouteEdge edge{delta, target_base.name, dst_wire};
                if (!target_dst_for_edge(source_base.name, src_wire, edge, source_cb_coord,
                                         final_target_base_id, dst_node,
                                         final_target_coord, final_target_wire, path)) {
                    continue;
                }
                Coord final_delta{
                    final_target_coord.x - source_cb_coord.x,
                    final_target_coord.y - source_cb_coord.y,
                };
                if (final_delta.x == 0 && final_delta.y == 0) {
                    continue;
                }
                const CBType& final_target_base = cb_types[final_target_base_id];
                src_node = source_node_for_resolved_delta(candidate, base_id, source_base,
                    src_node, src_wire, final_delta, path);
                if (final_target_base.type_id == base_id && dst_node >= 0) {
                    remember_synthetic_node(candidate, CB_NODE_DST, final_target_base.name,
                        final_target_wire, Coord{0,0}, dst_node);
                }
                add_mapping(candidate, overridden_src, src_node, final_target_base, dst_node, final_delta,
                            source_cb_coord, src_wire, final_target_wire);
            }
        };

        constexpr bool enable_rule_side_resolved = false;
        if constexpr (enable_rule_side_resolved) {
            for (const PendingRule& pending : pending_rules) {
                apply_rule_side_resolved(base, cb_types[pending.target_base_id],
                                         *pending.rule, pending.reverse);
                changed = true;
            }
        }

        if (tile.coord.x == source_cb_coord.x && tile.coord.y == source_cb_coord.y) {
            for (uint16_t src_node : tileconn_srcs_by_base[base_id]) {
                for (const std::string& src_wire : source_wires_for_src(base_id, src_node)) {
                if ((debugTileConnCoord(source_cb_coord) || debugTileConnWire(src_wire))
                    && (src_node == 1536 || src_node == 1768 || src_node == 1793)) {
                    auto raw_edges_it = route_wire_graph.find(tileConnKey(base.name, src_wire));
                    size_t raw_edges = raw_edges_it == route_wire_graph.end() ? 0 : raw_edges_it->second.size();
                    size_t valid_edges = valid_edges_from_tile_wire(base.name, src_wire, source_cb_coord, true).size();
                    PNR_LOG("FPGA",
                        "tileconn source alias coord=({}, {}) cb='{}' src={} wire='{}' raw_edges={} valid_edges={}",
                        source_cb_coord.x, source_cb_coord.y, base.name, src_node, src_wire, raw_edges, valid_edges);
                }
                for (const IncidentRouteEdge& edge : valid_edges_from_tile_wire(base.name, src_wire, source_cb_coord, true)) {
                    Coord target_coord{source_cb_coord.x + edge.delta.x, source_cb_coord.y + edge.delta.y};
                    std::string path = route_path_node(base.name, source_cb_coord, src_wire)
                        + " -> " + route_path_node(edge.tile_type, target_coord, edge.wire);
                    uint16_t final_target_base_id = CB_INVALID_TYPE_ID;
                    int dst_node = -1;
                    Coord final_target_coord{};
                    std::string final_target_wire;
                    if (!target_dst_for_edge(base.name, src_wire, edge, source_cb_coord,
                                             final_target_base_id, dst_node,
                                             final_target_coord, final_target_wire, path)) {
                        if (debugTileConnCoord(source_cb_coord) || debugTileConnWire(src_wire)) {
                            PNR_LOG("FPGA",
                                "tileconn resolve skip coord=({}, {}) source_cb='{}' src_wire='{}' first_hop='{}.{}' delta=({}, {}) path={}",
                                source_cb_coord.x, source_cb_coord.y, base.name, src_wire,
                                edge.tile_type, edge.wire, edge.delta.x, edge.delta.y, path);
                        }
                        continue;
                    }
                    Coord final_delta{
                        final_target_coord.x - source_cb_coord.x,
                        final_target_coord.y - source_cb_coord.y,
                    };
                    if (final_delta.x == 0 && final_delta.y == 0) {
                        continue;
                    }
                    const CBType& final_target_base = cb_types[final_target_base_id];
                    if (final_target_base.type_id == base_id) {
                        remember_synthetic_node(candidate, CB_NODE_DST, final_target_base.name,
                            final_target_wire, Coord{0,0}, dst_node);
                    }
                    int resolved_src_node = source_node_for_resolved_delta(candidate, base_id, base,
                        src_node, src_wire, final_delta, path);
                    NodeMask old_dst = candidate.dstMaskForSrc(src_node);
                    size_t old_entries = candidate.dst_by_src[src_node].size();
                    add_mapping(candidate, overridden_src, resolved_src_node, final_target_base,
                                dst_node, final_delta, source_cb_coord, src_wire, final_target_wire);
                    if (resolved_src_node != src_node
                        || candidate.dstMaskForSrc(src_node) != old_dst
                        || candidate.dst_by_src[src_node].size() != old_entries) {
                        changed = true;
                    }
                }
                }
            }
        }

        constexpr bool enable_non_cb_passthrough_subtypes = false;
        if (enable_non_cb_passthrough_subtypes && tile.tile_type && !exactCBTypeFor(cb_types, tile.tile_type->name)) {
            auto wires_it = route_wires_by_type.find(tile.tile_type->name);
            if (wires_it != route_wires_by_type.end()) {
                for (const std::string& landed_wire : wires_it->second) {
                    std::vector<IncidentRouteEdge> inbound_edges =
                        valid_edges_from_tile_wire(tile.tile_type->name, landed_wire, tile.coord);
                    if (inbound_edges.empty()) {
                        continue;
                    }
                    std::vector<InternalWirePath> reachable_wires =
                        internal_reachable_wires(tile.tile_type->name, tile.coord, landed_wire);
                    for (const IncidentRouteEdge& inbound : inbound_edges) {
                        Coord inbound_coord{tile.coord.x + inbound.delta.x, tile.coord.y + inbound.delta.y};
                        Tile* inbound_tile = tile_at_coord(inbound_coord);
                        CBType* inbound_exact = exactCBTypeFor(cb_types, inbound.tile_type);
                        if ((inbound_exact && !cbHasRouteGraph(*inbound_exact))
                            || (!inbound_exact && (!inbound_tile || !inbound_tile->tile_type
                                || (inbound_tile->tile_type->sites.empty()
                                    && inbound_tile->tile_type->elements.empty())))) {
                            continue;
                        }
                        Coord inbound_route_coord = inbound_coord;
                        if (inbound_tile && inbound_tile->cb_coord.x >= 0 && inbound_tile->cb_coord.y >= 0) {
                            inbound_route_coord = inbound_tile->cb_coord;
                        }
                        Coord landed_delta{
                            source_cb_coord.x - inbound_route_coord.x,
                            source_cb_coord.y - inbound_route_coord.y,
                        };
                        if (!signed4DeltaInRange(landed_delta)
                            || (landed_delta.x == 0 && landed_delta.y == 0)) {
                            continue;
                        }
                        struct OutChoice
                        {
                            std::string wire;
                            std::string internal_path;
                            IncidentRouteEdge edge;
                        };
                        std::vector<OutChoice> out_choices;
                        for (const InternalWirePath& outgoing_path : reachable_wires) {
                            const std::string& outgoing_wire = outgoing_path.wire;
                            for (const IncidentRouteEdge& outbound :
                                 valid_edges_from_tile_wire(tile.tile_type->name, outgoing_wire, tile.coord)) {
                                if (outgoing_wire == landed_wire
                                    && outbound.tile_type == inbound.tile_type
                                    && outbound.wire == inbound.wire
                                    && outbound.delta.x == inbound.delta.x
                                    && outbound.delta.y == inbound.delta.y) {
                                    continue;
                                }
                                CBType* exact_outbound = exactCBTypeFor(cb_types, outbound.tile_type);
                                if ((exact_outbound && !cbHasRouteGraph(*exact_outbound))
                                    || (!exact_outbound
                                        && !non_cb_edge_continues(tile.tile_type->name, outbound, tile.coord))) {
                                    continue;
                                }
                                out_choices.push_back(OutChoice{outgoing_wire, outgoing_path.path, outbound});
                            }
                        }
                        if (out_choices.empty()) {
                            continue;
                        }

                        std::string inbound_path = route_path_node(inbound.tile_type, inbound_coord, inbound.wire)
                            + " -> " + route_path_node(tile.tile_type->name, tile.coord, landed_wire);
                        auto full_path_for_choice = [&](const OutChoice& choice) {
                            const std::string landed_node = route_path_node(tile.tile_type->name, tile.coord, landed_wire);
                            std::string full_path = inbound_path;
                            if (choice.internal_path != landed_node) {
                                std::string internal_suffix = choice.internal_path;
                                const std::string prefix = landed_node + " -> ";
                                if (internal_suffix.rfind(prefix, 0) == 0) {
                                    internal_suffix = internal_suffix.substr(prefix.size());
                                }
                                full_path += " -> " + internal_suffix;
                            }
                            Coord outbound_coord{tile.coord.x + choice.edge.delta.x, tile.coord.y + choice.edge.delta.y};
                            full_path += " -> " + route_path_node(choice.edge.tile_type, outbound_coord, choice.edge.wire);
                            return full_path;
                        };
                        std::string dst_path = inbound_path + " continuations=[";
                        for (size_t choice_index = 0; choice_index < out_choices.size(); ++choice_index) {
                            if (choice_index != 0) {
                                dst_path += " ; ";
                            }
                            dst_path += full_path_for_choice(out_choices[choice_index]);
                        }
                        dst_path += "]";
                        int dst_node = materialize_synthetic_node(candidate, base_id, CB_NODE_DST,
                            tile.tile_type->name, landed_wire, landed_delta, dst_path);
                        if (dst_node < 0) {
                            continue;
                        }
                        for (const OutChoice& out_choice : out_choices) {
                            const std::string& outgoing_wire = out_choice.wire;
                            const IncidentRouteEdge& outbound = out_choice.edge;
                            std::string full_path = full_path_for_choice(out_choice);
                            uint16_t target_base_id = CB_INVALID_TYPE_ID;
                            int target_dst_node = -1;
                            Coord final_target_coord{};
                            std::string final_target_wire;
                            if (!target_dst_for_edge(tile.tile_type->name, outgoing_wire, outbound, tile.coord,
                                                     target_base_id, target_dst_node,
                                                     final_target_coord, final_target_wire, full_path)) {
                                continue;
                            }
                            Tile* target_tile = tile_at_coord(final_target_coord);
                            Coord target_route_coord = final_target_coord;
                            if (target_tile && target_tile->cb_coord.x >= 0 && target_tile->cb_coord.y >= 0) {
                                target_route_coord = target_tile->cb_coord;
                            }
                            Coord full_outbound_delta{
                                target_route_coord.x - source_cb_coord.x,
                                target_route_coord.y - source_cb_coord.y,
                            };
                            if (!signed4DeltaInRange(full_outbound_delta)) {
                                full_outbound_delta = Coord{
                                    final_target_coord.x - source_cb_coord.x,
                                    final_target_coord.y - source_cb_coord.y,
                                };
                            }
                            if (!signed4DeltaInRange(full_outbound_delta)) {
                                continue;
                            }
                            if (full_outbound_delta.x == 0 && full_outbound_delta.y == 0) {
                                continue;
                            }
                            Coord source_bucket_delta = full_outbound_delta;
                            int src_node = materialize_synthetic_node(candidate, base_id, CB_NODE_SRC,
                                tile.tile_type->name, outgoing_wire, source_bucket_delta, full_path);
                            if (src_node < 0) {
                                continue;
                            }
                            candidate.dst_src[dst_node].jump |= NodeMask{0,1} << src_node;
                            candidate.rememberConnName(CB_NODE_DST, dst_node, CB_NODE_SRC, src_node,
                                synthetic_node_name(tile.tile_type->name, landed_wire, landed_delta),
                                synthetic_node_name(tile.tile_type->name, outgoing_wire, source_bucket_delta));

                                if (!overridden_src[src_node]) {
                                    candidate.dst_by_src[src_node].clear();
                                    overridden_src[src_node] = true;
                                }
                                if (debugTileConnCoord(source_cb_coord) || debugTileConnWire(outgoing_wire)) {
                                    PNR_LOG("FPGA",
                                        "tileconn passthrough map coord=({}, {}) cb='{}' src={} '{}' delta=({}, {}) target_cb_id={} target_dst={} '{}' path={}",
                                        source_cb_coord.x, source_cb_coord.y, candidate.name, src_node,
                                        candidate.nodeName(CB_NODE_SRC, src_node) ? *candidate.nodeName(CB_NODE_SRC, src_node) : std::string{},
                                        full_outbound_delta.x, full_outbound_delta.y, target_base_id, target_dst_node,
                                        cb_types[target_base_id].nodeName(CB_NODE_DST, target_dst_node)
                                            ? *cb_types[target_base_id].nodeName(CB_NODE_DST, target_dst_node)
                                            : std::string{},
                                        full_path);
                                }
                                addResolvedJumpBit(candidate.dst_by_src[src_node], full_outbound_delta,
                                    target_base_id, target_dst_node, candidate.name, src_node,
                                    cb_types[target_base_id].name, final_target_wire, true);
                                addSrcPriorityDelta(candidate, src_node, full_outbound_delta);
                                candidate.derived_masks_valid = false;
                                changed = true;
                        }
                    }
                }
            }
        }

        if (debugTileConnCoord(tile.coord) || debugTileConnCoord(source_cb_coord)) {
            static constexpr int debug_srcs[] = {
                8, 9, 10, 11,
                16, 17, 18, 19,
                24, 25, 26, 27,
                56, 57, 58, 59,
                128, 129, 130, 131,
                132, 133, 134, 135,
                136, 137, 138, 139,
                256, 257, 258, 259,
                320, 321, 322, 323,
                368, 369, 370, 371,
                408, 409, 410, 411,
                488, 489, 490, 491,
                512, 513, 514, 515,
                1536, 1537, 1538, 1539,
                1768, 1769, 1770, 1771,
                1792, 1793, 1794, 1795,
                1808, 1809, 1810, 1811,
                896, 897, 898, 899,
                1904, 1905, 1906, 1907,
            };
            for (int src_node : debug_srcs) {
                const std::string* src_name = candidate.nodeName(CB_NODE_SRC, src_node);
                PNR_LOG("FPGA", "tileconn candidate coord=({}, {}) cb='{}' src={} '{}' dst_by_src_bits={} entries={}",
                    source_cb_coord.x, source_cb_coord.y, candidate.name, src_node,
                    src_name ? *src_name : std::string{},
                    countBits(candidate.dstMaskForSrc(src_node)), candidate.dst_by_src[src_node].size());
                for (const CBType::ResolvedJump& entry : candidate.dst_by_src[src_node]) {
                    PNR_LOG("FPGA", "tileconn candidate coord=({}, {}) src={} entry delta=({}, {}) target_cb_id={} dst_bits={}",
                        source_cb_coord.x, source_cb_coord.y, src_node, entry.delta.x, entry.delta.y,
                        entry.target_cb_type_id, countBits(entry.dsts.jump));
                }
            }
        }

        if (!changed) {
            assigned_type_ids[index] = base_id;
            subtype_by_topology_signature.emplace(std::move(signature), base_id);
            continue;
        }

        candidate.base_type_id = base_id;
        candidate.rebuildOutgoingSrcs();
        uint16_t reused_id = CB_INVALID_TYPE_ID;
        if (base_id < variants_by_base.size()) {
            for (uint16_t variant_id : variants_by_base[base_id]) {
                if (variant_id >= cb_types.size()) {
                    continue;
                }
                if (candidate.sameRoutingSubtype(cb_types[variant_id])) {
                    reused_id = variant_id;
                    break;
                }
            }
        }
        if (reused_id != CB_INVALID_TYPE_ID) {
            assigned_type_ids[index] = reused_id;
            subtype_by_topology_signature.emplace(std::move(signature), reused_id);
            if (reused_id != base_id) {
                ++specialized_tiles;
            }
            continue;
        }

        PNR_ASSERT(cb_types.size() < CB_INVALID_TYPE_ID, "too many CB subtypes: {}", cb_types.size());
        uint16_t chosen_id = static_cast<uint16_t>(cb_types.size());
        candidate.type_id = chosen_id;
        const CBType* old_cb_storage = cb_types.data();
        cb_types.push_back(std::move(candidate));
        variants_by_base[base_id].push_back(chosen_id);
        ++created_subtypes;
        assigned_type_ids[index] = chosen_id;
        if (old_cb_storage != cb_types.data()) {
            for (size_t tile_index = 0; tile_index < tile_grid.size(); ++tile_index) {
                uint16_t tile_type_id = assigned_type_ids[tile_index] != CB_INVALID_TYPE_ID
                    ? assigned_type_ids[tile_index]
                    : base_type_ids[tile_index];
                if (tile_type_id == CB_INVALID_TYPE_ID || tile_type_id >= cb_types.size()) {
                    continue;
                }
                tile_grid[tile_index].cb_type = &cb_types[tile_type_id];
                tile_grid[tile_index].cb.type = tile_grid[tile_index].cb_type;
            }
        }
        assigned_type_ids[index] = chosen_id;
        subtype_by_topology_signature.emplace(std::move(signature), chosen_id);
        ++specialized_tiles;
    }

    std::unordered_map<AttachedCbKey, uint16_t, AttachedCbKeyHash> type_by_route_cb;
    type_by_route_cb.reserve(tile_grid.size());
    for (size_t index = 0; index < tile_grid.size(); ++index) {
        Tile& tile = tile_grid[index];
        uint16_t type_id = assigned_type_ids[index];
        if (type_id == CB_INVALID_TYPE_ID) {
            type_id = base_type_ids[index];
        }
        if (type_id == CB_INVALID_TYPE_ID || type_id >= cb_types.size()
            || tile.cb_coord.x < 0 || tile.cb_coord.y < 0
            || tile.coord.x != tile.cb_coord.x || tile.coord.y != tile.cb_coord.y) {
            continue;
        }
        type_by_route_cb[AttachedCbKey{cbBaseTypeId(&cb_types[type_id]), tile.cb_coord.x, tile.cb_coord.y}] = type_id;
    }
    for (size_t index = 0; index < tile_grid.size(); ++index) {
        if (assigned_type_ids[index] != CB_INVALID_TYPE_ID) {
            continue;
        }
        Tile& tile = tile_grid[index];
        uint16_t base_id = base_type_ids[index];
        if (base_id == CB_INVALID_TYPE_ID || base_id >= cb_types.size()
            || tile.cb_coord.x < 0 || tile.cb_coord.y < 0) {
            continue;
        }
        auto it = type_by_route_cb.find(AttachedCbKey{cbBaseTypeId(&cb_types[base_id]), tile.cb_coord.x, tile.cb_coord.y});
        assigned_type_ids[index] = it != type_by_route_cb.end() ? it->second : base_id;
    }

    for (size_t index = 0; index < tile_grid.size(); ++index) {
        uint16_t type_id = assigned_type_ids[index];
        if (type_id == CB_INVALID_TYPE_ID) {
            continue;
        }
        Tile& tile = tile_grid[index];
        tile.cb_type = &cb_types[type_id];
        tile.cb.type = tile.cb_type;
    }

    for (CBType& cb_type : cb_types) {
        pruneUnresolvedRouteSources(cb_type);
    }

    size_t invalid_src_count = 0;
    for (const CBType& cb_type : cb_types) {
        NodeMask used_srcs = jumpSourcesUsedByRouteMasks(cb_type);
        used_srcs.for_each_set_bit([&](int src_node) {
            if (!cb_type.dst_by_src[src_node].empty()) {
                return false;
            }
            if (invalid_src_count < 64) {
                const std::string* src_name = cb_type.nodeName(CB_NODE_SRC, src_node);
                PNR_LOG("FPGA",
                    "tileconn invalid unresolved source: cb='{}' type_id={} base_type_id={} src={} '{}'",
                    cb_type.name, cb_type.type_id, cbBaseTypeId(&cb_type), src_node,
                    src_name ? *src_name : std::string{});
            }
            ++invalid_src_count;
            return false;
        });
    }
    if (invalid_src_count != 0) {
        PNR_LOG("FPGA", "tileconn invalid unresolved source total={}", invalid_src_count);
    }

    PNR_LOG("FPGA", "applyTileConnSubtypes created {} CBType subtypes for {} specialized tiles; sig_hit={} sig_miss={} target_calls={} target_hit={} target_ok={} target_fail={} bfs_pops={} edge_calls={} edge_returned={}",
        created_subtypes, specialized_tiles,
        signature_cache_hits, signature_cache_misses,
        target_dst_calls, target_dst_cache_hits, target_dst_success, target_dst_failed,
        target_dst_bfs_pops, valid_edges_calls, valid_edges_returned);
}

void Device::loadTypeFromSpec(const std::string& spec_name, TechMap& map)
{
    // any types except cb
    PNR_LOG("FPGA", "loadTypeFromSpec, spec_name: '{}'", spec_name);
    TileTypesSpec spec;
    std::map<std::string,TypeSpec> types;
    readTypes(spec_name, &types, &spec);
    for (const auto& type_spec : types) {
        auto existing = std::find_if(tile_types.begin(), tile_types.end(), [&](const TileType& type) {
            return type.name == type_spec.first;
        });
        TileType* type = nullptr;
        if (existing == tile_types.end()) {
            tile_types.push_back(TileType{type_spec.first, tile_types.size(), 0});
            type = &tile_types.back();
        }
        else {
            type = &*existing;
        }

        type->sites.clear();
        type->pin_map = {};
        for (const TypeSpec::SiteSpec& site_spec : type_spec.second.sites) {
            SiteModel site;
            site.name = site_spec.name;
            site.type = site_spec.type;
            site.pos = site_spec.pos;
            site.pins = site_spec.pins;
            type->sites.push_back(std::move(site));
        }

        for (const TypeSpec::WireEdgeSpec& edge : type_spec.second.wire_edges) {
            // Tile-internal wires extend endpoint discovery without changing route traversal.
            addRouteWireGraphEdge(route_wire_graph, type_spec.first, edge.src, type_spec.first, edge.dst);
            addRouteWireGraphEdge(route_wire_graph, type_spec.first, edge.dst, type_spec.first, edge.src);
        }

        type->rebuildElementsFromSites();
        bool resolve_pin_map = type_spec.second.direct_site_wire_endpoints
            || (!type_spec.second.input_pins.empty() || !type_spec.second.output_pins.empty())
            || (!type->elements.empty() && typeSpecHasPackableSite(type_spec.second));
        if (!resolve_pin_map) {
            PNR_LOG1("FPGA", "loadTypeFromSpec, loaded '{}' with {} input and {} output resource nodes, {} local wire names and {} resource names",
                type->name, type->pin_map.input_nodes.size(), type->pin_map.output_nodes.size(),
                type->pin_map.local_wire_names.size(), type->pin_map.local_resource_names.size());
            continue;
        }

        LocalNodeUse input_node_use = type_spec.second.direct_site_wire_endpoints ? LocalNodeUse::input : LocalNodeUse::any;
        LocalNodeUse output_node_use = type_spec.second.direct_site_wire_endpoints ? LocalNodeUse::output : LocalNodeUse::any;
        std::unordered_map<std::string, std::vector<LocalNodeMapping>> resolved_local_node_cache;
        auto resolve_mappings = [&](const std::string& tile_type_name, const std::string& wire, LocalNodeUse use) {
            std::string key = tileConnKey(tile_type_name, wire) + "\n" + std::to_string(static_cast<int>(use));
            auto cache_it = resolved_local_node_cache.find(key);
            if (cache_it != resolved_local_node_cache.end()) {
                return cache_it->second;
            }
            std::vector<LocalNodeMapping> mappings = resolveLocalNodeMappings(
                cb_types, local_route_wire_mappings, route_wire_graph, tile_type_name, wire, use);
            auto [inserted, _] = resolved_local_node_cache.emplace(std::move(key), std::move(mappings));
            return inserted->second;
        };

        for (const TypeSpec::PinNodeSpec& pin : type_spec.second.input_pins) {
            int resource_node = resourceNodeFromMap(map, pin.port, pin.pos);
            if (resource_node < 0) {
                continue;
            }
            type->pin_map.rememberResourcePinName(TILE_PIN_INPUT, resource_node, pin.port);
            for (const std::string& wire : pin.nodes) {
                std::vector<LocalNodeMapping> direct_mappings = resolve_mappings(type_spec.first, wire, input_node_use);
                std::vector<LocalNodeMapping> unfiltered_mappings;
                for (const LocalNodeMapping& mapping : direct_mappings) {
                    if (!localNodeMatchesUse(cb_types, mapping, LocalNodeUse::input)) {
                        continue;
                    }
                    int local_node = mapping.local_node;
                    // Keep input resource pins mapped only to locals that can be entered from routing.
                    type->pin_map.input_nodes[resource_node] |= NodeMask{0,1} << local_node;
                    type->pin_map.rememberLocalNames(TILE_PIN_INPUT, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointNames(TILE_PIN_INPUT, resource_node, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointRouteRef(TILE_PIN_INPUT, resource_node, local_node,
                                                           mapping.route_type, mapping.delta);
                }
                if (technology::mappedRouteEndpointAliases) {
                    for (const auto& alias : technology::mappedRouteEndpointAliases(type_spec.first, pin.port, pin.pos, wire)) {
                        auto route_delta_for_alias = [&](const LocalNodeMapping& mapping) {
                            Coord route_delta = mapping.delta;
                            const std::vector<LocalNodeMapping>* base_mappings = &direct_mappings;
                            auto base_it = std::find_if(base_mappings->begin(), base_mappings->end(),
                                [&](const LocalNodeMapping& base) { return base.route_type == alias.first; });
                            if (base_it == base_mappings->end()) {
                                if (unfiltered_mappings.empty()) {
                                    unfiltered_mappings = resolve_mappings(type_spec.first, wire, LocalNodeUse::any);
                                }
                                base_mappings = &unfiltered_mappings;
                            }
                            for (const LocalNodeMapping& direct_mapping : *base_mappings) {
                                if (direct_mapping.route_type == alias.first) {
                                    route_delta = Coord{direct_mapping.delta.x + mapping.delta.x,
                                                        direct_mapping.delta.y + mapping.delta.y};
                                    break;
                                }
                            }
                            return route_delta;
                        };
                        std::vector<LocalNodeMapping> alias_input_mappings = resolve_mappings(alias.first, alias.second, input_node_use);
                        std::vector<LocalNodeMapping> alias_output_mappings = resolve_mappings(alias.first, alias.second, output_node_use);
                        if (debugEndpointAlias(type_spec.first, pin.port)) {
                            PNR_LOG("FPGA", "endpoint alias input-pin tile='{}' pin='{}' pos={} wire='{}' alias='{}:{}' input_maps={} output_maps={} direct_maps={} unfiltered_pending={}",
                                type_spec.first, pin.port, pin.pos, wire, alias.first, alias.second,
                                alias_input_mappings.size(), alias_output_mappings.size(), direct_mappings.size(),
                                unfiltered_mappings.empty() ? 0 : unfiltered_mappings.size());
                            for (const LocalNodeMapping& mapping : direct_mappings) {
                                PNR_LOG("FPGA", "endpoint alias direct resource={} route='{}' local={} delta=({}, {})",
                                    resource_node, mapping.route_type, mapping.local_node, mapping.delta.x, mapping.delta.y);
                            }
                            for (const LocalNodeMapping& mapping : alias_input_mappings) {
                                PNR_LOG("FPGA", "endpoint alias input-candidate route='{}' local={} delta=({}, {})",
                                    mapping.route_type, mapping.local_node, mapping.delta.x, mapping.delta.y);
                            }
                            for (const LocalNodeMapping& mapping : alias_output_mappings) {
                                PNR_LOG("FPGA", "endpoint alias output-candidate route='{}' local={} delta=({}, {})",
                                    mapping.route_type, mapping.local_node, mapping.delta.x, mapping.delta.y);
                            }
                        }
                        for (const LocalNodeMapping& mapping : alias_input_mappings) {
                            if (!localNodeMatchesUse(cb_types, mapping, LocalNodeUse::input)) {
                                continue;
                            }
                            int local_node = mapping.local_node;
                            Coord route_delta = route_delta_for_alias(mapping);
                            if (debugEndpointAlias(type_spec.first, pin.port)) {
                                PNR_LOG("FPGA", "endpoint alias store INPUT resource={} local={} route='{}' delta=({}, {})",
                                    resource_node, local_node, mapping.route_type, route_delta.x, route_delta.y);
                            }
                            type->pin_map.input_nodes[resource_node] |= NodeMask{0,1} << local_node;
                            type->pin_map.rememberLocalNames(TILE_PIN_INPUT, local_node, alias.second, pin.wire, pin.port);
                            type->pin_map.rememberEndpointNames(TILE_PIN_INPUT, resource_node, local_node, alias.second, pin.wire, pin.port);
                            type->pin_map.rememberEndpointRouteRef(TILE_PIN_INPUT, resource_node, local_node,
                                                                   mapping.route_type, route_delta);
                        }
                        for (const LocalNodeMapping& mapping : alias_output_mappings) {
                            if (!localNodeMatchesUse(cb_types, mapping, LocalNodeUse::output)) {
                                continue;
                            }
                            int local_node = mapping.local_node;
                            Coord route_delta = route_delta_for_alias(mapping);
                            if (debugEndpointAlias(type_spec.first, pin.port)) {
                                PNR_LOG("FPGA", "endpoint alias store OUTPUT resource={} local={} route='{}' delta=({}, {})",
                                    resource_node, local_node, mapping.route_type, route_delta.x, route_delta.y);
                            }
                            type->pin_map.output_nodes[resource_node] |= NodeMask{0,1} << local_node;
                            type->pin_map.rememberLocalNames(TILE_PIN_OUTPUT, local_node, alias.second, pin.wire, pin.port);
                            type->pin_map.rememberEndpointNames(TILE_PIN_OUTPUT, resource_node, local_node, alias.second, pin.wire, pin.port);
                            type->pin_map.rememberEndpointRouteRef(TILE_PIN_OUTPUT, resource_node, local_node,
                                                                   mapping.route_type, route_delta);
                        }
                    }
                }
            }
        }

        for (const TypeSpec::PinNodeSpec& pin : type_spec.second.output_pins) {
            int resource_node = resourceNodeFromMap(map, pin.port, pin.pos);
            if (resource_node < 0) {
                continue;
            }
            type->pin_map.rememberResourcePinName(TILE_PIN_OUTPUT, resource_node, pin.port);
            for (const std::string& wire : pin.nodes) {
                std::vector<LocalNodeMapping> direct_mappings = resolve_mappings(type_spec.first, wire, output_node_use);
                std::vector<LocalNodeMapping> unfiltered_mappings;
                for (const LocalNodeMapping& mapping : direct_mappings) {
                    if (!localNodeMatchesUse(cb_types, mapping, LocalNodeUse::output)) {
                        continue;
                    }
                    int local_node = mapping.local_node;
                    // Keep output resource pins mapped only to locals that can launch into routing.
                    type->pin_map.output_nodes[resource_node] |= NodeMask{0,1} << local_node;
                    type->pin_map.rememberLocalNames(TILE_PIN_OUTPUT, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointNames(TILE_PIN_OUTPUT, resource_node, local_node, wire, pin.wire, pin.port);
                    type->pin_map.rememberEndpointRouteRef(TILE_PIN_OUTPUT, resource_node, local_node,
                                                           mapping.route_type, mapping.delta);
                }
                if (technology::mappedRouteEndpointAliases) {
                    for (const auto& alias : technology::mappedRouteEndpointAliases(type_spec.first, pin.port, pin.pos, wire)) {
                        auto route_delta_for_alias = [&](const LocalNodeMapping& mapping) {
                            Coord route_delta = mapping.delta;
                            const std::vector<LocalNodeMapping>* base_mappings = &direct_mappings;
                            auto base_it = std::find_if(base_mappings->begin(), base_mappings->end(),
                                [&](const LocalNodeMapping& base) { return base.route_type == alias.first; });
                            if (base_it == base_mappings->end()) {
                                if (unfiltered_mappings.empty()) {
                                    unfiltered_mappings = resolve_mappings(type_spec.first, wire, LocalNodeUse::any);
                                }
                                base_mappings = &unfiltered_mappings;
                            }
                            for (const LocalNodeMapping& direct_mapping : *base_mappings) {
                                if (direct_mapping.route_type == alias.first) {
                                    route_delta = Coord{direct_mapping.delta.x + mapping.delta.x,
                                                        direct_mapping.delta.y + mapping.delta.y};
                                    break;
                                }
                            }
                            return route_delta;
                        };
                        std::vector<LocalNodeMapping> alias_output_mappings = resolve_mappings(alias.first, alias.second, output_node_use);
                        std::vector<LocalNodeMapping> alias_input_mappings = resolve_mappings(alias.first, alias.second, input_node_use);
                        if (debugEndpointAlias(type_spec.first, pin.port)) {
                            PNR_LOG("FPGA", "endpoint alias output-pin tile='{}' pin='{}' pos={} wire='{}' alias='{}:{}' input_maps={} output_maps={} direct_maps={} unfiltered_pending={}",
                                type_spec.first, pin.port, pin.pos, wire, alias.first, alias.second,
                                alias_input_mappings.size(), alias_output_mappings.size(), direct_mappings.size(),
                                unfiltered_mappings.empty() ? 0 : unfiltered_mappings.size());
                            for (const LocalNodeMapping& mapping : direct_mappings) {
                                PNR_LOG("FPGA", "endpoint alias direct resource={} route='{}' local={} delta=({}, {})",
                                    resource_node, mapping.route_type, mapping.local_node, mapping.delta.x, mapping.delta.y);
                            }
                            for (const LocalNodeMapping& mapping : alias_output_mappings) {
                                PNR_LOG("FPGA", "endpoint alias output-candidate route='{}' local={} delta=({}, {})",
                                    mapping.route_type, mapping.local_node, mapping.delta.x, mapping.delta.y);
                            }
                            for (const LocalNodeMapping& mapping : alias_input_mappings) {
                                PNR_LOG("FPGA", "endpoint alias input-candidate route='{}' local={} delta=({}, {})",
                                    mapping.route_type, mapping.local_node, mapping.delta.x, mapping.delta.y);
                            }
                        }
                        for (const LocalNodeMapping& mapping : alias_output_mappings) {
                            if (!localNodeMatchesUse(cb_types, mapping, LocalNodeUse::output)) {
                                continue;
                            }
                            int local_node = mapping.local_node;
                            Coord route_delta = route_delta_for_alias(mapping);
                            if (debugEndpointAlias(type_spec.first, pin.port)) {
                                PNR_LOG("FPGA", "endpoint alias store OUTPUT resource={} local={} route='{}' delta=({}, {})",
                                    resource_node, local_node, mapping.route_type, route_delta.x, route_delta.y);
                            }
                            type->pin_map.output_nodes[resource_node] |= NodeMask{0,1} << local_node;
                            type->pin_map.rememberLocalNames(TILE_PIN_OUTPUT, local_node, alias.second, pin.wire, pin.port);
                            type->pin_map.rememberEndpointNames(TILE_PIN_OUTPUT, resource_node, local_node, alias.second, pin.wire, pin.port);
                            type->pin_map.rememberEndpointRouteRef(TILE_PIN_OUTPUT, resource_node, local_node,
                                                                   mapping.route_type, route_delta);
                        }
                        for (const LocalNodeMapping& mapping : alias_input_mappings) {
                            if (!localNodeMatchesUse(cb_types, mapping, LocalNodeUse::input)) {
                                continue;
                            }
                            int local_node = mapping.local_node;
                            Coord route_delta = route_delta_for_alias(mapping);
                            if (debugEndpointAlias(type_spec.first, pin.port)) {
                                PNR_LOG("FPGA", "endpoint alias store INPUT resource={} local={} route='{}' delta=({}, {})",
                                    resource_node, local_node, mapping.route_type, route_delta.x, route_delta.y);
                            }
                            type->pin_map.input_nodes[resource_node] |= NodeMask{0,1} << local_node;
                            type->pin_map.rememberLocalNames(TILE_PIN_INPUT, local_node, alias.second, pin.wire, pin.port);
                            type->pin_map.rememberEndpointNames(TILE_PIN_INPUT, resource_node, local_node, alias.second, pin.wire, pin.port);
                            type->pin_map.rememberEndpointRouteRef(TILE_PIN_INPUT, resource_node, local_node,
                                                                   mapping.route_type, route_delta);
                        }
                    }
                }
            }
        }
        PNR_LOG1("FPGA", "loadTypeFromSpec, loaded '{}' with {} input and {} output resource nodes, {} local wire names and {} resource names",
            type->name, type->pin_map.input_nodes.size(), type->pin_map.output_nodes.size(),
            type->pin_map.local_wire_names.size(), type->pin_map.local_resource_names.size());
    }
}

void Device::loadCBFromSpec(const std::string& spec_name, TechMap& map)
{
    // crossbars
    PNR_LOG("FPGA", "loadCBFromSpec, spec_name: '{}'", spec_name);
    TileTypesSpec spec;
    std::map<std::string,CBTypeSpec> cbs;
    readCBTypes(spec_name, &cbs, &spec);
    for (auto& cb : cbs) {
        if (CBType* existing = exactCBTypeFor(cb_types, cb.first)) {
            existing->type_id = cbTypeIdFor(cb_types, existing);
            existing->base_type_id = existing->type_id;
            PNR_LOG1("FPGA", "loadCBFromSpec, updating cb_type '{}' ptr={}", cb.first, static_cast<const void*>(existing));
            existing->loadFromSpec(cb.second, map);
            continue;
        }
        cb_types.emplace_back(cb.first);
        PNR_ASSERT(cb_types.size() - 1 < CB_INVALID_TYPE_ID, "too many CB types: {}\n", cb_types.size());
        cb_types.back().type_id = static_cast<uint16_t>(cb_types.size() - 1);
        cb_types.back().base_type_id = cb_types.back().type_id;
        PNR_LOG1("FPGA", "loadCBFromSpec, inserting cb_type '{}' ptr={}", cb.first, static_cast<const void*>(&cb_types.back()));
        cb_types.back().loadFromSpec(cb.second, map);
    }
    const char* debug_dst_text = std::getenv("SCALEPNR_DEBUG_CB_DST");
    if (debug_dst_text && *debug_dst_text) {
        int debug_dst = atoi(debug_dst_text);
        int debug_local = -1;
        const char* debug_local_text = std::getenv("SCALEPNR_DEBUG_CB_LOCAL");
        if (debug_local_text && *debug_local_text) {
            debug_local = atoi(debug_local_text);
        }
        if (debug_dst >= 0 && debug_dst < CB_MAX_NODES) {
            for (const CBType& cb_type : cb_types) {
                bool local_bit = debug_local >= 0 && debug_local < CB_MAX_NODES
                    && (cb_type.dst_local[debug_dst].local & (NodeMask{0,1} << debug_local)) != NodeMask{};
                NodeMask dst_local = cb_type.dst_local[debug_dst].local;
                PNR_LOG1("FPGA", "loadCBFromSpec debug cb_type '{}' ptr={} dst={} local={} local_bit={} dst_local={}",
                    cb_type.name, static_cast<const void*>(&cb_type), debug_dst, debug_local, local_bit,
                    dst_local.str());
            }
        }
    }
}

void Device::loadTileConnFromSpec(const std::string& spec_name)
{
    PNR_LOG("FPGA", "loadTileConnFromSpec, spec_name: '{}'", spec_name);
    std::ifstream infile(spec_name);
    if (!infile) {
        throw std::runtime_error(std::string("cant open file: ") + spec_name);
    }

    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(infile, root)) {
        throw std::runtime_error(std::string("cant parse tileconn JSON: ") + spec_name);
    }

    local_route_wire_mappings.clear();
    route_wire_graph.clear();
    std::vector<ParsedTileConnRule> rules;
    for (const Json::Value& item : root) {
        if (!item.isMember("tile_types") || !item.isMember("wire_pairs")) {
            continue;
        }
        const Json::Value& types = item["tile_types"];
        if (!types.isArray() || types.size() != 2) {
            continue;
        }

        ParsedTileConnRule rule;
        rule.from_tile_type = types[0].asString();
        rule.to_tile_type = types[1].asString();
        if (item.isMember("grid_deltas") && item["grid_deltas"].isArray() && item["grid_deltas"].size() == 2) {
            rule.delta = Coord{item["grid_deltas"][0].asInt(), item["grid_deltas"][1].asInt()};
        }
        for (const Json::Value& pair : item["wire_pairs"]) {
            if (!pair.isArray() || pair.size() != 2) {
                continue;
            }
            ParsedTileConnPair parsed_pair{pair[0].asString(), pair[1].asString()};
            addRouteWireGraphEdge(route_wire_graph, rule.from_tile_type, parsed_pair.from_wire,
                                  rule.to_tile_type, parsed_pair.to_wire, rule.delta, true);
            addRouteWireGraphEdge(route_wire_graph, rule.to_tile_type, parsed_pair.to_wire,
                                  rule.from_tile_type, parsed_pair.from_wire, Coord{-rule.delta.x, -rule.delta.y}, true);
            rule.wire_pairs.push_back(std::move(parsed_pair));
        }
        rules.push_back(std::move(rule));
    }

    size_t local_mappings = 0;
    for (const ParsedTileConnRule& rule : rules) {
        const CBType* from_cb = exactCBTypeFor(cb_types, rule.from_tile_type);
        const CBType* to_cb = exactCBTypeFor(cb_types, rule.to_tile_type);
        for (const ParsedTileConnPair& pair : rule.wire_pairs) {
            if (from_cb && !to_cb) {
                local_route_wire_mappings[tileConnKey(rule.to_tile_type, pair.to_wire)].push_back(
                    LocalRouteWireMapping{rule.from_tile_type, pair.from_wire, Coord{-rule.delta.x, -rule.delta.y}});
                ++local_mappings;
            }
            if (to_cb && !from_cb) {
                local_route_wire_mappings[tileConnKey(rule.from_tile_type, pair.from_wire)].push_back(
                    LocalRouteWireMapping{rule.to_tile_type, pair.to_wire, rule.delta});
                ++local_mappings;
            }
        }
    }

    tileconn_rules = std::move(rules);

    PNR_LOG("FPGA", "loadTileConnFromSpec loaded {} tile connection rules and {} local pin mappings",
        tileconn_rules.size(), local_mappings);
}

Tile* Device::getTile(int x, int y)
{
    if (x < 0 || y < 0 || x >= size_width || y >= size_height) {
        return nullptr;
    }
    return &tile_grid[y*grid_spec.size.x + x];
}

TileJumpTarget Device::resolveJump(const Tile& from, int src_node) const
{
    if (!from.cb_type || src_node < 0 || src_node >= CB_MAX_NODES) {
        return {};
    }
    static const Referable<Tile>* cached_grid_data = nullptr;
    static size_t cached_grid_size = 0;
    static std::unordered_map<ResolveJumpCacheKey, TileJumpTarget, ResolveJumpCacheKeyHash> cache;
    if (cached_grid_data != tile_grid.data() || cached_grid_size != tile_grid.size()) {
        cached_grid_data = tile_grid.data();
        cached_grid_size = tile_grid.size();
        cache.clear();
    }
    ResolveJumpCacheKey cache_key{
        &from,
        from.cb_type,
        src_node
    };
    if (auto cache_it = cache.find(cache_key); cache_it != cache.end()) {
        return cache_it->second;
    }
    bool debug = debugResolveJumpCoord(from.coord);

    const auto& exact = from.cb_type->dst_by_src[src_node];
    if (debug) {
        const std::string* src_name = from.cb_type->nodeName(CB_NODE_SRC, src_node);
        PNR_LOG("FPGA", "resolveJump start from=({}, {}) tile='{}' cb='{}' src={} '{}' decoder_entries={} mask_candidates={}",
            from.coord.x, from.coord.y, from.name, from.cb_type->name, src_node,
            src_name ? *src_name : std::string{},
            exact.size(), countBits(from.cb_type->dstMaskForSrc(src_node)));
    }
    if (!exact.empty()) {
        for (const CBType::ResolvedJump& entry : exact) {
            TileJumpTarget target = resolvedJumpTarget(*this, from, src_node, entry.delta,
                                                       entry.target_cb_type_id, entry.dsts.jump,
                                                       entry.target_tile_coord, &entry.dst_wires, debug);
            if (!target.tile) {
                continue;
            }
            cache[cache_key] = target;
            return target;
        }
        cache[cache_key] = {};
        return {};
    }

    cache[cache_key] = {};
    return {};
}

std::vector<TileJumpTarget> Device::resolveJumpTargets(const Tile& from, int src_node) const
{
    std::vector<TileJumpTarget> targets;
    if (!from.cb_type || src_node < 0 || src_node >= CB_MAX_NODES) {
        return targets;
    }
    bool debug = debugResolveJumpCoord(from.coord);
    const auto& exact = from.cb_type->dst_by_src[src_node];
    for (const CBType::ResolvedJump& entry : exact) {
        appendResolvedJumpTargets(*this, from, src_node, entry.delta,
                                  entry.target_cb_type_id, entry.dsts.jump,
                                  entry.target_tile_coord, &entry.dst_wires, debug, targets);
    }
    return targets;
}

TileJumpTarget Device::resolveJumpToward(const Tile& from, int src_node, const Coord& target) const
{
    if (!from.cb_type || src_node < 0 || src_node >= CB_MAX_NODES) {
        return {};
    }
    static const Referable<Tile>* cached_grid_data = nullptr;
    static size_t cached_grid_size = 0;
    static std::unordered_map<ResolveJumpTowardCacheKey, TileJumpTarget, ResolveJumpTowardCacheKeyHash> cache;
    if (cached_grid_data != tile_grid.data() || cached_grid_size != tile_grid.size()) {
        cached_grid_data = tile_grid.data();
        cached_grid_size = tile_grid.size();
        cache.clear();
    }
    ResolveJumpTowardCacheKey cache_key{
        ResolveJumpCacheKey{
            &from,
            from.cb_type,
            src_node
        },
        target.x,
        target.y
    };
    if (auto cache_it = cache.find(cache_key); cache_it != cache.end()) {
        return cache_it->second;
    }

    const auto& exact = from.cb_type->dst_by_src[src_node];
    if (exact.empty()) {
        cache[cache_key] = {};
        return {};
    }

    Coord target_bucket = jumpTargetBucket(target - from.coord);
    bool debug = debugResolveJumpCoord(from.coord);
    TileJumpTarget best;
    PriorityRank best_rank;
    for (const CBType::ResolvedJump& entry : exact) {
        TileJumpTarget candidate = resolvedJumpTarget(*this, from, src_node, entry.delta,
                                                      entry.target_cb_type_id, entry.dsts.jump,
                                                      entry.target_tile_coord, &entry.dst_wires, debug);
        if (!candidate.tile) {
            continue;
        }
        PriorityRank rank = rankForDelta(candidate.tile->coord - from.coord, target_bucket, src_node);
        if (!best.tile || rankBefore(rank, best_rank)) {
            best = candidate;
            best_rank = rank;
        }
    }

    cache[cache_key] = best;
    return best;
}
