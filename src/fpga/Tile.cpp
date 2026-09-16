#include "Tile.h"
#include "Device.h"
#include "Net.h"
#include "Wire.h"
#include "RegBunch.h"
#include "Timings.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdio>
#include <cstdlib>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

using namespace fpga;

bool Tile::hasRoutedNet(rtl::Net* net) const
{
    if (routed_net_index_size != routedNets.size()
        || routed_net_index_data != routedNets.data()) {
        routed_net_index.clear();
        routed_net_index.reserve(routedNets.size());
        for (const Ref<rtl::Net>& ref : routedNets) {
            if (ref.peer) {
                routed_net_index.insert(ref.peer);
            }
        }
        routed_net_index_size = routedNets.size();
        routed_net_index_data = routedNets.data();
    }
    return net && routed_net_index.contains(net);
}

void Tile::addRoutedNet(rtl::Net* net)
{
    if (!net || hasRoutedNet(net)) {
        return;
    }
    for (Ref<rtl::Net>& ref : routedNets) {
        if (!ref.peer) {
            ref.set(static_cast<Referable<rtl::Net>*>(net));
            routed_net_index.insert(net);
            return;
        }
    }
    Ref<rtl::Net>& ref = routedNets.emplace_back();
    ref.set(static_cast<Referable<rtl::Net>*>(net));
    routed_net_index.insert(net);
    routed_net_index_size = routedNets.size();
    routed_net_index_data = routedNets.data();
}

void Tile::removeRoutedNet(rtl::Net* net)
{
    if (!net || !hasRoutedNet(net)) {
        return;
    }
    for (Ref<rtl::Net>& ref : routedNets) {
        if (ref.peer == net) {
            ref.clear();
        }
    }
    routed_net_index.erase(net);
}

void Tile::clearRoutedNets()
{
    routedNets.clear();
    routed_bindings.clear();
    routed_bindings_authoritative = false;
    routed_net_index.clear();
    routed_net_index_size = routedNets.size();
    routed_net_index_data = routedNets.data();
}

void Tile::addRoutedBinding(rtl::Net* net, uint64_t route_id)
{
    if (!net || route_id == 0) {
        return;
    }
    routed_bindings_authoritative = true;
    auto found = std::find_if(routed_bindings.begin(), routed_bindings.end(),
        [&](const RoutedBinding& old) {
            return old.net == net && old.route_id == route_id;
        });
    if (found == routed_bindings.end()) {
        routed_bindings.push_back(RoutedBinding{net, route_id});
    }
    addRoutedNet(net);
}

void Tile::removeRoutedBindings(rtl::Net* net)
{
    if (!net) {
        return;
    }
    std::erase_if(routed_bindings,
                  [&](const RoutedBinding& ref) { return ref.net == net; });
    removeRoutedNet(net);
}

namespace technology {
#if defined(__GNUC__)
std::string mappedSitePinName(const std::string& cell_type, const std::string& port,
                              int pos, const std::string& fallback) __attribute__((weak));
#else
std::string mappedSitePinName(const std::string& cell_type, const std::string& port,
                              int pos, const std::string& fallback);
#endif
}

namespace {

constexpr uint16_t bit16(int index)
{
    return index >= 0 && index < ELEMENT_BITMAP_BITS ? static_cast<uint16_t>(1u << index) : 0;
}

int elementColumn(ElementType type)
{
    return static_cast<int>(type);
}

ElementType instElementType(const rtl::Inst& inst)
{
    // Map generic primitive classes into the abstract tile element columns.
    static const std::string empty_type;
    const std::string& type = inst.cell_ref.peer ? inst.cell_ref.peer->type : empty_type;
    if (type.find("FD") == 0) {
        return ELEMENT_FD;
    }
    if (type.find("CARRY") == 0) {
        return ELEMENT_CARRY;
    }
    if (type.find("MUXF8") == 0) {
        return ELEMENT_MUXF8;
    }
    if (type.find("MUX") == 0) {
        return ELEMENT_MUXF7;
    }
    if (type == "INV") {
        return ELEMENT_LUT5;
    }
    if (type == "LUT1" || inst.cnt_inputs == 1) {
        return ELEMENT_LUT1;
    }
    return ELEMENT_LUT5;
}

std::optional<ElementType> maybeInstElementType(const rtl::Inst& inst)
{
    // Only real placeable primitive classes participate in element-chain checks.
    static const std::string empty_type;
    const std::string& type = inst.cell_ref.peer ? inst.cell_ref.peer->type : empty_type;
    if (type.find("FD") == 0) {
        return ELEMENT_FD;
    }
    if (type.find("CARRY") == 0) {
        return ELEMENT_CARRY;
    }
    if (type.find("MUXF8") == 0) {
        return ELEMENT_MUXF8;
    }
    if (type.find("MUX") == 0) {
        return ELEMENT_MUXF7;
    }
    if (type == "INV") {
        return ELEMENT_LUT5;
    }
    if (type == "LUT1" || type.find("LUT") == 0) {
        return type == "LUT1" ? ELEMENT_LUT1 : ELEMENT_LUT5;
    }
    return std::nullopt;
}

bool isLutElement(ElementType type)
{
    return type == ELEMENT_LUT5 || type == ELEMENT_LUT1;
}

bool isFullLut6(const rtl::Inst& inst)
{
    static const std::string empty_type;
    const std::string& type = inst.cell_ref.peer ? inst.cell_ref.peer->type : empty_type;
    return type == "LUT6" || inst.cnt_inputs >= 6;
}

void reserveElementBit(Tile& tile, ElementType type, int bit, const rtl::Inst* inst)
{
    // Full LUT6 consumes the paired LUT+1 lane; smaller LUTs may still share it.
    if (bit < 0 || bit >= ELEMENT_BITMAP_BITS) {
        return;
    }
    tile.elements_free[type] &= static_cast<uint16_t>(~bit16(bit));
    if (type == ELEMENT_LUT5 && inst && isFullLut6(*inst)
        && (tile.elements_pos[ELEMENT_LUT1] & bit16(bit)) != 0) {
        tile.elements_free[ELEMENT_LUT1] &= static_cast<uint16_t>(~bit16(bit));
    }
}

int placedPosFromElementBit(ElementType type, int bit)
{
    // Encode an element bitmap lane back into the existing abstract CLB position.
    switch (type) {
    case ELEMENT_FD: {
        int site = bit / 8;
        int lane = bit % 8;
        int bel = lane % 4;
        int fd_column = lane >= 4 ? 64 : 0;
        return site*128 + fd_column + bel*4;
    }
    case ELEMENT_LUT5:
    case ELEMENT_LUT1:
    case ELEMENT_CARRY:
    case ELEMENT_MUXF7:
    case ELEMENT_MUXF8: {
        int site = bit / 4;
        int bel = bit % 4;
        if (type == ELEMENT_CARRY) {
            return site*128 + 2;
        }
        if (type == ELEMENT_MUXF7) {
            return site*128 + bel*4 + 1;
        }
        if (type == ELEMENT_MUXF8) {
            return site*128 + 1;
        }
        return site*128 + bel*4 + 3;
    }
    default:
        return -1;
    }
}

bool isLogicLanePin(const std::string& port, char prefix)
{
    if (port.empty() || port[0] != prefix) {
        return false;
    }
    return port.size() == 1
        || (port.size() == 2 && (port[1] == 'X' || port[1] == 'Q'));
}

bool siteHasLogicLane(const SiteModel& site, char prefix)
{
    return std::any_of(site.pins.begin(), site.pins.end(), [&](const Pin& pin) {
        return isLogicLanePin(pin.port, prefix);
    });
}

bool siteHasPort(const SiteModel& site, const std::string& port);

bool siteHasLogicOutputLanes(const SiteModel& site)
{
    static constexpr char prefixes[4] = {'A', 'B', 'C', 'D'};
    for (char prefix : prefixes) {
        if (siteHasPort(site, std::string{prefix} + "X") || siteHasPort(site, std::string{prefix} + "Q")) {
            return true;
        }
    }
    return false;
}

bool siteHasPort(const SiteModel& site, const std::string& port)
{
    return std::any_of(site.pins.begin(), site.pins.end(), [&](const Pin& pin) {
        return pin.port == port;
    });
}

uint16_t detectedSiteBelMask(const SiteModel& site)
{
    // Database site pins decide which abstract BEL lanes physically exist.
    static constexpr char prefixes[4] = {'A', 'B', 'C', 'D'};
    uint16_t mask = 0;
    for (int bel = 0; bel < 4; ++bel) {
        if (siteHasLogicLane(site, prefixes[bel])) {
            mask |= bit16(bel);
        }
    }
    return mask;
}

void addElement(TileType& type, const std::string& name, ElementType element_type, uint16_t bit, int column)
{
    Element element;
    element.name = name;
    element.type = element_type;
    element.bitmap_pos = bit;
    element.elements_to_left = column;
    type.elements.push_back(std::move(element));
}

void connectElements(TileType& type, ElementType left_type, uint16_t left_bit, ElementType right_type, uint16_t right_bit)
{
    // Add directional neighbor connectivity between adjacent element columns.
    for (Element& element : type.elements) {
        if (element.type == left_type && element.bitmap_pos == left_bit) {
            element.right_blockers[right_bit] |= bit16(left_bit);
        }
        if (element.type == right_type && element.bitmap_pos == right_bit) {
            element.left_blockers[left_bit] |= bit16(right_bit);
        }
    }
}

thread_local rtl::Inst* pack_debug_context = nullptr;
thread_local bool pack_debug_context_enabled = false;
thread_local bool enforce_pack_route_capacity = true;

struct PackDebugScope
{
    explicit PackDebugScope(rtl::Inst* inst)
        : previous(pack_debug_context), previous_enabled(pack_debug_context_enabled)
    {
        pack_debug_context = inst;
        pack_debug_context_enabled = false;
        if (std::getenv("SCALEPNR_PACK_DEBUG") == nullptr) {
            return;
        }
        const char* filter = std::getenv("SCALEPNR_PACK_DEBUG_INST");
        pack_debug_context_enabled = !filter || filter[0] == '\0'
            || (inst && inst->makeName(std::numeric_limits<size_t>::max()).find(filter)
                != std::string::npos);
    }

    ~PackDebugScope()
    {
        pack_debug_context = previous;
        pack_debug_context_enabled = previous_enabled;
    }

    rtl::Inst* previous = nullptr;
    bool previous_enabled = false;
};

bool packDebugEnabled()
{
    if (pack_debug_context) {
        return pack_debug_context_enabled;
    }
    return std::getenv("SCALEPNR_PACK_DEBUG") != nullptr
        && std::getenv("SCALEPNR_PACK_DEBUG_INST") == nullptr;
}

void printTypeMasks(const char* prefix, const std::array<uint16_t, ELEMENT_TYPE_COUNT>& masks)
{
    std::fprintf(stderr, "%s", prefix);
    for (int type_index = 0; type_index < ELEMENT_TYPE_COUNT; ++type_index) {
        ElementType type = static_cast<ElementType>(type_index);
        std::fprintf(stderr, " %s=0x%04x", elementTypeName(type), masks[type_index]);
    }
    std::fprintf(stderr, "\n");
}

void printElementLinks(const TileType& type)
{
    for (const Element& element : type.elements) {
        std::fprintf(stderr, "pack-debug type=%s element=%s bit=%u column=%d",
            type.name.c_str(), element.name.c_str(), element.bitmap_pos, element.elements_to_left);
        for (int bit = 0; bit < ELEMENT_BITMAP_BITS; ++bit) {
            if (element.left_blockers[bit]) {
                std::fprintf(stderr, " L[%d]=0x%04x", bit, element.left_blockers[bit]);
            }
            if (element.right_blockers[bit]) {
                std::fprintf(stderr, " R[%d]=0x%04x", bit, element.right_blockers[bit]);
            }
        }
        std::fprintf(stderr, "\n");
    }
}

int extractIndexedPort(std::string& port)
{
    size_t open = port.find('[');
    if (open == std::string::npos || port.back() != ']') {
        return -1;
    }

    std::string bit = port.substr(open + 1, port.size() - open - 2);
    port = port.substr(0, open);
    try {
        return std::stoi(bit);
    }
    catch (...) {
        return -1;
    }
}

int belIndexFromPlacedPos(int pos)
{
    while (pos >= 128) {
        pos -= 128;
    }
    return (pos % 64) / 4;
}

int muxDataBelFromPlacedPos(const std::string& type, const std::string& port, int pos)
{
    // MUX data inputs occupy adjacent lanes; keep I0 and I1 distinct.
    int bel = belIndexFromPlacedPos(pos);
    if (type.find("MUXF7") == 0) {
        int base = bel < 2 ? 0 : 2;
        return port == "I1" ? std::min(base + 1, 3) : base;
    }
    if (type.find("MUXF8") == 0) {
        return port == "I1" ? 2 : 0;
    }
    return bel;
}

int muxControlBelFromPlacedPos(const std::string& type, int pos)
{
    // MUX select pins are anchored to the control lane of the selected mux.
    int bel = belIndexFromPlacedPos(pos);
    if (type.find("MUXF7") == 0) {
        return bel < 2 ? 0 : 2;
    }
    if (type.find("MUXF8") == 0) {
        return 1;
    }
    return bel;
}

int muxOutputBelFromPlacedPos(const std::string& type, int pos)
{
    // A wide mux outputs through the lane between its two narrow-mux inputs.
    return type.find("MUXF8") == 0
        ? muxControlBelFromPlacedPos(type, pos)
        : muxDataBelFromPlacedPos(type, "I0", pos);
}

int siteIndexFromPlacedPos(int pos)
{
    return pos >= 0 ? pos / 128 : 0;
}

int elementBitFromPlacedPos(ElementType type, int pos)
{
    // Decode the abstract CLB position into the element bitmap lane.
    int site = siteIndexFromPlacedPos(pos);
    int bel = belIndexFromPlacedPos(pos);
    if (site < 0 || bel < 0 || bel > 3) {
        return -1;
    }
    if (type == ELEMENT_FD) {
        int fd_column = (pos % 128) >= 64 ? 1 : 0;
        int fd_bit = site*8 + fd_column*4 + bel;
        return fd_bit < ELEMENT_BITMAP_BITS ? fd_bit : -1;
    }
    if (site*4 + bel >= ELEMENT_BITMAP_BITS) {
        return -1;
    }
    switch (type) {
    case ELEMENT_MUXF8:
    case ELEMENT_CARRY:
        bel = 0;
        break;
    case ELEMENT_MUXF7:
        bel = bel < 2 ? 0 : 2;
        break;
    default:
        break;
    }
    int bit = site*4 + bel;
    return bit < ELEMENT_BITMAP_BITS ? bit : -1;
}

int belIndexFromBitOrPos(int bit, int pos)
{
    if (bit >= 0 && bit < 4) {
        return bit;
    }
    return belIndexFromPlacedPos(pos);
}

int indexedNode(const int nodes[4], int index)
{
    return index >= 0 && index < 4 ? nodes[index] : -1;
}

rtl::Inst* instFromTileRef(RefBase<Referable<Tile>>* ref)
{
    // Recover the owning instance from a Tile reference peer.
    auto* tile_ref = Ref<Tile>::fromBase(ref);
    return reinterpret_cast<rtl::Inst*>(reinterpret_cast<char*>(tile_ref) - offsetof(rtl::Inst, tile));
}

bool useResourcePinNameFallback(const std::string& type);
std::string modeledResourcePinName(const TileType* tile_type, std::string type, std::string port, int pos);
int modeledSitePos(const TileType* tile_type, int pos);
int modeledResourceNodeNum(const TileType* tile_type, const std::string& type, int pos, int base_node);

std::vector<rtl::Inst*> assignedInsts(Tile& tile)
{
    // Enumerate all instances currently placed into this tile.
    std::vector<rtl::Inst*> insts;
    auto& referable_tile = static_cast<Referable<Tile>&>(tile);
    for (auto* peer : referable_tile.getPeers()) {
        if (!peer) {
            continue;
        }
        rtl::Inst* inst = instFromTileRef(peer);
        if (inst && inst->tile.peer == &referable_tile) {
            insts.push_back(inst);
        }
    }
    return insts;
}

bool isLut(const rtl::Inst& inst)
{
    // Classify every primitive mapped to an abstract LUT element as LUT logic.
    std::optional<ElementType> type = maybeInstElementType(inst);
    return type && isLutElement(*type);
}

bool isCarry(const rtl::Inst& inst)
{
    // Classify carry primitives by generic cell type prefix.
    return inst.cell_ref.peer && inst.cell_ref.peer->type.find("CARRY") == 0;
}

bool isMux(const rtl::Inst& inst)
{
    // Classify mux primitives by generic cell type prefix.
    return inst.cell_ref.peer && inst.cell_ref.peer->type.find("MUX") == 0;
}

rtl::Module* parentModule(rtl::Inst& inst)
{
    // Resolve the parent module that owns this instance's flat nets.
    if (!inst.cell_ref.peer || !inst.cell_ref->module_ref.peer) {
        return nullptr;
    }
    return inst.cell_ref->module_ref->parent_ref.peer;
}

struct ModuleNetDesignatorIndex
{
    const void* nets_data = nullptr;
    size_t indexed_nets = 0;
    int next_generated = 100000000;
    std::unordered_map<int, size_t> net_by_designator;
};

ModuleNetDesignatorIndex& moduleNetDesignatorIndex(rtl::Module& module)
{
    // Generated passthrough nets only append to the module.  Index those
    // additions incrementally instead of rescanning every net for every task.
    static std::unordered_map<rtl::Module*, ModuleNetDesignatorIndex> indexes;
    ModuleNetDesignatorIndex& index = indexes[&module];
    const void* nets_data = module.nets.empty() ? nullptr : module.nets.data();
    if (index.nets_data != nets_data || index.indexed_nets > module.nets.size()) {
        index = {};
        index.nets_data = nets_data;
    }
    while (index.indexed_nets < module.nets.size()) {
        size_t net_index = index.indexed_nets++;
        for (int designator : module.nets[net_index].designators) {
            index.net_by_designator.try_emplace(designator, net_index);
            index.next_generated = std::max(index.next_generated, designator + 1);
        }
    }
    return index;
}

rtl::Net* findNetByDesignator(rtl::Inst& inst, int designator)
{
    // Find the flat net attached to a local connection designator.
    rtl::Module* parent = parentModule(inst);
    if (!parent) {
        return nullptr;
    }
    ModuleNetDesignatorIndex& index = moduleNetDesignatorIndex(*parent);
    auto found = index.net_by_designator.find(designator);
    if (found != index.net_by_designator.end()
        && found->second < parent->nets.size()) {
        return &parent->nets[found->second];
    }
    return nullptr;
}

bool canHost(Tile& tile, rtl::Inst* inst, int pos);
rtl::Inst* elementInstAt(Tile& tile, ElementType type, int bit);
bool futureStrictOutputSinksFit(Tile& tile, rtl::Inst& future_inst, ElementType future_type,
                                int future_bit,
                                const std::array<uint16_t, ELEMENT_TYPE_COUNT>& reserved);
bool placedStrictPeerUsesLane(Tile& tile, ElementType type, int bit, ElementType peer_type,
                              int peer_bit, bool peer_on_right);

bool portBitMatches(const rtl::Port& port, const std::string& name, int bit)
{
    // Compare scalar and indexed ports against a normalized bit selector.
    std::string port_name = port.name;
    int port_bit = extractIndexedPort(port_name);
    if (port_bit < 0) {
        port_bit = port.bitnum;
    }
    return port_name == name && port_bit == bit;
}

rtl::Conn* carryInputDriver(rtl::Inst& inst, const std::string& name, int bit)
{
    // Find the driver connected to one carry input bit.
    if (!isCarry(inst)) {
        return nullptr;
    }
    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        if (!portBitMatches(*conn.port_ref, name, bit)) {
            continue;
        }
        return conn.follow();
    }
    return nullptr;
}

bool hasOutputConn(rtl::Inst& inst, rtl::Conn* driver)
{
    // Check whether an instance output is the selected driver connection.
    if (!driver) {
        return false;
    }
    for (auto& conn : inst.conns) {
        if (conn.port_ref.peer && conn.port_ref->type == rtl::Port::PORT_OUT && &conn == driver) {
            return true;
        }
    }
    return false;
}

bool drivesInput(rtl::Inst& driver, rtl::Inst& sink)
{
    // True when one resource output directly feeds another resource input.
    for (auto& conn : sink.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        rtl::Conn* followed = conn.follow();
        if (followed && followed->inst_ref.peer == &driver && followed->port_ref.peer
            && followed->port_ref->type == rtl::Port::PORT_OUT) {
            return true;
        }
    }
    return false;
}

bool connectedInOrder(rtl::Inst& left, rtl::Inst& right)
{
    // Neighbor columns are directional: left element output must drive right input.
    return drivesInput(left, right);
}

bool sameAssignedTile(Tile& tile, rtl::Inst& inst)
{
    return inst.tile.peer == &static_cast<Referable<Tile>&>(tile);
}

bool strictLocalChainPair(ElementType left, ElementType right)
{
    // These element links are dedicated tile-local arcs, not general fabric routes.
    return (isLutElement(left) && right == ELEMENT_MUXF7)
        || (left == ELEMENT_MUXF7 && right == ELEMENT_MUXF8);
}

bool strictLocalChainInput(ElementType left, ElementType right, const rtl::Port* sink_port)
{
    // Only mux data inputs are strict tile-local arcs; selectors route as normal inputs.
    if (!sink_port || !strictLocalChainPair(left, right)) {
        return false;
    }
    return sink_port->name == "I0" || sink_port->name == "I1";
}

bool strictLocalChainLaneMatches(ElementType driver_type, int driver_bit,
                                 ElementType sink_type, int sink_bit,
                                 std::string_view sink_port)
{
    // Ordered mux data ports select distinct lanes among the loaded adjacent element links.
    if (sink_port != "I0" && sink_port != "I1") {
        return true;
    }
    if (isLutElement(driver_type) && sink_type == ELEMENT_MUXF7) {
        int expected = sink_port == "I0" ? sink_bit + 1 : sink_bit;
        return driver_bit == expected;
    }
    if (driver_type == ELEMENT_MUXF7 && sink_type == ELEMENT_MUXF8) {
        int expected = sink_port == "I0" ? sink_bit + 2 : sink_bit;
        return driver_bit == expected;
    }
    return true;
}

bool strictLocalChainLaneMatches(ElementType driver_type, int driver_bit,
                                 ElementType sink_type, int sink_bit,
                                 const rtl::Port* sink_port)
{
    if (!strictLocalChainInput(driver_type, sink_type, sink_port)) {
        return true;
    }
    return strictLocalChainLaneMatches(driver_type, driver_bit, sink_type, sink_bit, sink_port->name);
}

bool strictLocalChainReachable(rtl::Inst& source, rtl::Inst& target, int depth = 0)
{
    // Strict-chain aliases may be transitive, for example LUT -> MUXF7 -> MUXF8.
    if (depth > 4) {
        return false;
    }
    std::optional<ElementType> source_type_opt = maybeInstElementType(source);
    if (!source_type_opt) {
        return false;
    }
    ElementType source_type = *source_type_opt;
    for (rtl::Conn& output : source.conns) {
        if (!output.port_ref.peer || output.port_ref->type != rtl::Port::PORT_OUT
            || output.peer) {
            continue;
        }
        for (auto* sink_ref : rtl::Conn::getSinks(output)) {
            rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            rtl::Inst* sink = sink_conn ? sink_conn->inst_ref.peer : nullptr;
            if (!sink || !sink->cell_ref.peer) {
                continue;
            }
            std::optional<ElementType> sink_type_opt = maybeInstElementType(*sink);
            if (!sink_type_opt || !strictLocalChainInput(source_type, *sink_type_opt,
                                                         sink_conn ? sink_conn->port_ref.peer : nullptr)) {
                continue;
            }
            if (sink == &target || strictLocalChainReachable(*sink, target, depth + 1)) {
                return true;
            }
        }
    }
    return false;
}

bool strictSinkHasPlacedDriverOutside(Tile& tile, rtl::Inst& sink, rtl::Inst& candidate_driver)
{
    // All strict-chain inputs of one unplaced sink must converge on one tile.
    std::optional<ElementType> sink_type_opt = maybeInstElementType(sink);
    if (!sink_type_opt) {
        return false;
    }
    ElementType sink_type = *sink_type_opt;
    for (rtl::Conn& input : sink.conns) {
        if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        rtl::Conn* driver_conn = input.follow();
        rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
        if (!driver || driver == &candidate_driver || !driver->cell_ref.peer || !driver->tile.peer) {
            continue;
        }
        std::optional<ElementType> driver_type_opt = maybeInstElementType(*driver);
        if (!driver_type_opt) {
            continue;
        }
        ElementType driver_type = *driver_type_opt;
        if (strictLocalChainInput(driver_type, sink_type, input.port_ref.peer) && !sameAssignedTile(tile, *driver)) {
            if (packDebugEnabled()) {
                std::fprintf(stderr,
                    "pack-debug   sibling strict driver outside tile sink=%s driver=%s driver_tile=%s driver_coord=(%d,%d) candidate=%s tile=%s coord=(%d,%d)\n",
                    sink.makeName().c_str(), driver->makeName().c_str(),
                    driver->tile.peer ? driver->tile->makeName().c_str() : "-",
                    driver->tile.peer ? driver->tile->coord.x : -1,
                    driver->tile.peer ? driver->tile->coord.y : -1,
                    candidate_driver.makeName().c_str(), tile.makeName().c_str(), tile.coord.x, tile.coord.y);
            }
            return true;
        }
    }
    return false;
}

uint16_t linkedNeighborMask(Tile& tile, ElementType type, int bit, ElementType neighbor_type,
                            bool right_side)
{
    // Return physical neighbor lanes connected to this element bit.
    const auto& links = right_side ? tile.elements_right[type] : tile.elements_left[type];
    const auto& reciprocal = right_side ? tile.elements_left[neighbor_type] : tile.elements_right[neighbor_type];
    uint16_t mask = 0;
    for (int neighbor_bit = 0; neighbor_bit < ELEMENT_BITMAP_BITS; ++neighbor_bit) {
        if ((links[neighbor_bit] & bit16(bit)) == 0) {
            continue;
        }
        if ((reciprocal[bit] & bit16(neighbor_bit)) == 0) {
            continue;
        }
        PNR_ASSERT((tile.elements_pos[neighbor_type] & bit16(neighbor_bit)) != 0,
            "element {} bit {} references missing neighbor {} bit {} in tile type {}",
            elementTypeName(type), bit, elementTypeName(neighbor_type), neighbor_bit,
            tile.tile_type ? tile.tile_type->name : "?");
        mask |= bit16(neighbor_bit);
    }
    return mask;
}

bool futureStrictInputDriversFit(Tile& tile, rtl::Inst& future_inst, ElementType future_type,
                                 int future_bit,
                                 const std::array<uint16_t, ELEMENT_TYPE_COUNT>& reserved)
{
    // A reserved future mux lane is usable only if its own strict input drivers can share that lane.
    for (rtl::Conn& input : future_inst.conns) {
        if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        rtl::Conn* driver_conn = input.follow();
        rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
        if (!driver || !driver->cell_ref.peer) {
            continue;
        }
        std::optional<ElementType> driver_type_opt = maybeInstElementType(*driver);
        if (!driver_type_opt) {
            continue;
        }
        ElementType driver_type = *driver_type_opt;
        if (!strictLocalChainInput(driver_type, future_type, input.port_ref.peer)) {
            continue;
        }
        if (driver->tile.peer) {
            if (!sameAssignedTile(tile, *driver)) {
                return false;
            }
            int driver_bit = elementBitFromPlacedPos(driver_type, driver->pos);
            if (driver_bit < 0
                || (linkedNeighborMask(tile, future_type, future_bit, driver_type, false) & bit16(driver_bit)) == 0
                || !strictLocalChainLaneMatches(driver_type, driver_bit, future_type, future_bit, input.port_ref.peer)) {
                return false;
            }
            continue;
        }

        uint16_t available = linkedNeighborMask(tile, future_type, future_bit, driver_type, false);
        available &= tile.elements_free[driver_type];
        available &= static_cast<uint16_t>(~reserved[driver_type]);
        bool driver_fits = false;
        while (available) {
            int driver_bit = std::countr_zero(static_cast<unsigned>(available));
            available &= static_cast<uint16_t>(available - 1);
            int driver_pos = placedPosFromElementBit(driver_type, driver_bit);
            if (driver_pos >= 0 && canHost(tile, driver, driver_pos)
                && strictLocalChainLaneMatches(driver_type, driver_bit, future_type, future_bit, input.port_ref.peer)) {
                driver_fits = true;
                break;
            }
        }
        if (!driver_fits) {
            if (packDebugEnabled()) {
                std::fprintf(stderr,
                    "pack-debug     shared-lane future-strict-driver-no-fit future=%s driver=%s future_bit=%d type=%s\n",
                    future_inst.makeName().c_str(), driver->makeName().c_str(), future_bit,
                    elementTypeName(driver_type));
            }
            return false;
        }
    }
    return true;
}

bool outputLocalCompatible(Tile& tile, rtl::Inst* inst, ElementType type, int bit);
bool inputLocalCompatible(Tile& tile, rtl::Inst* inst, ElementType type, int bit);

bool futureOccupiedBlockersCompatible(Tile& tile, rtl::Inst& future_inst, ElementType future_type,
                                      int future_bit)
{
    // Reserved future lanes cannot cross already occupied linked elements from unrelated cells.
    for (bool left_side : {true, false}) {
        const auto& links = left_side ? tile.elements_left[future_type] : tile.elements_right[future_type];
        for (int type_index = 0; type_index < ELEMENT_TYPE_COUNT; ++type_index) {
            ElementType neighbor_type = static_cast<ElementType>(type_index);
            const auto& reciprocal = left_side ? tile.elements_right[neighbor_type] : tile.elements_left[neighbor_type];
            for (int neighbor_bit = 0; neighbor_bit < ELEMENT_BITMAP_BITS; ++neighbor_bit) {
                if ((links[neighbor_bit] & bit16(future_bit)) == 0) {
                    continue;
                }
                if ((reciprocal[future_bit] & bit16(neighbor_bit)) == 0) {
                    continue;
                }
                if ((tile.elements_pos[neighbor_type] & bit16(neighbor_bit)) == 0
                    || (tile.elements_free[neighbor_type] & bit16(neighbor_bit)) != 0) {
                    continue;
                }
                rtl::Inst* neighbor = elementInstAt(tile, neighbor_type, neighbor_bit);
                if (!neighbor) {
                    return false;
                }
                if (left_side) {
                    if (!connectedInOrder(*neighbor, future_inst)) {
                        return false;
                    }
                }
                else if (!connectedInOrder(future_inst, *neighbor)) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool futureOccupiedInputBlockersCompatible(Tile& tile, rtl::Inst& future_inst,
                                           ElementType future_type, int future_bit)
{
    // A future sink must not inherit an unrelated occupied predecessor lane.
    int predecessor_column = elementColumn(future_type) - 1;
    const auto& links = tile.elements_left[future_type];
    for (int type_index = 0; type_index < ELEMENT_TYPE_COUNT; ++type_index) {
        ElementType predecessor_type = static_cast<ElementType>(type_index);
        if (elementColumn(predecessor_type) != predecessor_column) {
            continue;
        }
        const auto& reciprocal = tile.elements_right[predecessor_type];
        for (int predecessor_bit = 0; predecessor_bit < ELEMENT_BITMAP_BITS; ++predecessor_bit) {
            if ((links[predecessor_bit] & bit16(future_bit)) == 0
                || (reciprocal[future_bit] & bit16(predecessor_bit)) == 0
                || (tile.elements_pos[predecessor_type] & bit16(predecessor_bit)) == 0
                || (tile.elements_free[predecessor_type] & bit16(predecessor_bit)) != 0) {
                continue;
            }
            rtl::Inst* predecessor = elementInstAt(tile, predecessor_type, predecessor_bit);
            if (!predecessor || !connectedInOrder(*predecessor, future_inst)) {
                return false;
            }
        }
    }
    return true;
}

bool hasSharedFreeSinkLane(Tile& tile, rtl::Inst& sink, rtl::Inst& candidate_driver,
                           ElementType candidate_type, int candidate_bit,
                           bool require_sink_host = true)
{
    // Multiple drivers of one future strict-chain sink must share one legal sink lane.
    std::optional<ElementType> sink_type_opt = maybeInstElementType(sink);
    if (!sink_type_opt) {
        return false;
    }
    ElementType sink_type = *sink_type_opt;
    uint16_t common = linkedNeighborMask(tile, candidate_type, candidate_bit, sink_type, true);
    const rtl::Port* candidate_sink_port = nullptr;
    for (rtl::Conn& input : sink.conns) {
        rtl::Conn* driver_conn = input.follow();
        if (driver_conn && driver_conn->inst_ref.peer == &candidate_driver) {
            candidate_sink_port = input.port_ref.peer;
            break;
        }
    }
    if (common == 0) {
        if (packDebugEnabled()) {
            std::fprintf(stderr, "pack-debug     shared-lane none candidate=%s bit=%d sink=%s sink_type=%s\n",
                candidate_driver.makeName().c_str(), candidate_bit, sink.makeName().c_str(),
                elementTypeName(sink_type));
        }
        return false;
    }
    for (rtl::Conn& input : sink.conns) {
        if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        rtl::Conn* driver_conn = input.follow();
        rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
        if (!driver || driver == &candidate_driver || !driver->cell_ref.peer || !driver->tile.peer) {
            continue;
        }
        std::optional<ElementType> driver_type_opt = maybeInstElementType(*driver);
        if (!driver_type_opt) {
            continue;
        }
        ElementType driver_type = *driver_type_opt;
        if (!strictLocalChainInput(driver_type, sink_type, input.port_ref.peer)) {
            continue;
        }
        if (!sameAssignedTile(tile, *driver)) {
            if (packDebugEnabled()) {
                std::fprintf(stderr, "pack-debug     shared-lane placed-driver-other-tile driver=%s sink=%s\n",
                    driver->makeName().c_str(), sink.makeName().c_str());
            }
            return false;
        }
        int driver_bit = elementBitFromPlacedPos(driver_type, driver->pos);
        if (driver_bit < 0 || driver_bit >= ELEMENT_BITMAP_BITS) {
            if (packDebugEnabled()) {
                std::fprintf(stderr, "pack-debug     shared-lane bad-driver-bit driver=%s pos=%d sink=%s\n",
                    driver->makeName().c_str(), driver->pos, sink.makeName().c_str());
            }
            return false;
        }
        common &= linkedNeighborMask(tile, driver_type, driver_bit, sink_type, true);
        if (common == 0) {
            if (packDebugEnabled()) {
                std::fprintf(stderr, "pack-debug     shared-lane no-common-placed driver=%s bit=%d sink=%s\n",
                    driver->makeName().c_str(), driver_bit, sink.makeName().c_str());
            }
            return false;
        }
    }
    common &= tile.elements_free[sink_type];
    if (packDebugEnabled()) {
        std::fprintf(stderr, "pack-debug     shared-lane common-free sink=%s mask=0x%04x\n",
            sink.makeName().c_str(), common);
    }
    while (common) {
        int sink_bit = std::countr_zero(static_cast<unsigned>(common));
        common &= static_cast<uint16_t>(common - 1);
        if (!strictLocalChainLaneMatches(candidate_type, candidate_bit, sink_type, sink_bit,
                                         candidate_sink_port)) {
            continue;
        }
        bool placed_drivers_match = true;
        for (rtl::Conn& input : sink.conns) {
            rtl::Conn* driver_conn = input.follow();
            rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
            if (!driver || !driver->tile.peer || driver == &candidate_driver || !driver->cell_ref.peer) {
                continue;
            }
            std::optional<ElementType> driver_type_opt = maybeInstElementType(*driver);
            if (!driver_type_opt || !strictLocalChainInput(*driver_type_opt, sink_type, input.port_ref.peer)) {
                continue;
            }
            int driver_bit = elementBitFromPlacedPos(*driver_type_opt, driver->pos);
            if (!strictLocalChainLaneMatches(*driver_type_opt, driver_bit, sink_type, sink_bit, input.port_ref.peer)) {
                placed_drivers_match = false;
                break;
            }
        }
        if (!placed_drivers_match) {
            continue;
        }
        int sink_pos = placedPosFromElementBit(sink_type, sink_bit);
        if (sink_pos < 0 || (require_sink_host && !canHost(tile, &sink, sink_pos))) {
            if (packDebugEnabled()) {
                std::fprintf(stderr, "pack-debug     shared-lane reject-sink-host sink=%s bit=%d pos=%d\n",
                    sink.makeName().c_str(), sink_bit, sink_pos);
            }
            continue;
        }
        if (!inputLocalCompatible(tile, &sink, sink_type, sink_bit)
            || !outputLocalCompatible(tile, &sink, sink_type, sink_bit)) {
            if (packDebugEnabled()) {
                std::fprintf(stderr, "pack-debug     shared-lane reject-sink-local sink=%s bit=%d pos=%d\n",
                    sink.makeName().c_str(), sink_bit, sink_pos);
            }
            continue;
        }
        if (!futureOccupiedInputBlockersCompatible(tile, sink, sink_type, sink_bit)) {
            if (packDebugEnabled()) {
                std::fprintf(stderr,
                    "pack-debug     shared-lane reject-sink-blocker sink=%s bit=%d pos=%d\n",
                    sink.makeName().c_str(), sink_bit, sink_pos);
            }
            continue;
        }

        std::array<uint16_t, ELEMENT_TYPE_COUNT> reserved{};
        bool all_future_drivers_fit = true;
        for (rtl::Conn& input : sink.conns) {
            if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) {
                continue;
            }
            rtl::Conn* driver_conn = input.follow();
            rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
            if (!driver || driver == &candidate_driver || !driver->cell_ref.peer || driver->tile.peer) {
                continue;
            }
            std::optional<ElementType> driver_type_opt = maybeInstElementType(*driver);
            if (!driver_type_opt) {
                continue;
            }
            ElementType driver_type = *driver_type_opt;
            if (!strictLocalChainInput(driver_type, sink_type, input.port_ref.peer)) {
                continue;
            }
            uint16_t available = linkedNeighborMask(tile, sink_type, sink_bit, driver_type, false);
            available &= tile.elements_free[driver_type];
            available &= static_cast<uint16_t>(~reserved[driver_type]);
            if (driver_type == candidate_type) {
                available &= static_cast<uint16_t>(~bit16(candidate_bit));
            }

            if (packDebugEnabled()) {
                std::fprintf(stderr, "pack-debug     shared-lane future-driver sink_bit=%d driver=%s type=%s available=0x%04x reserved=0x%04x candidate_bit=%d\n",
                    sink_bit, driver->makeName().c_str(), elementTypeName(driver_type), available,
                    reserved[driver_type], driver_type == candidate_type ? candidate_bit : -1);
            }

            bool driver_fits = false;
            while (available) {
                int driver_bit = std::countr_zero(static_cast<unsigned>(available));
                available &= static_cast<uint16_t>(available - 1);
                int driver_pos = placedPosFromElementBit(driver_type, driver_bit);
                if (driver_pos >= 0 && canHost(tile, driver, driver_pos)
                    && strictLocalChainLaneMatches(driver_type, driver_bit, sink_type, sink_bit, input.port_ref.peer)
                    && futureOccupiedBlockersCompatible(tile, *driver, driver_type, driver_bit)
                    && futureStrictInputDriversFit(tile, *driver, driver_type, driver_bit, reserved)) {
                    reserved[driver_type] |= bit16(driver_bit);
                    driver_fits = true;
                    break;
                }
                if (packDebugEnabled()) {
                    std::fprintf(stderr, "pack-debug     shared-lane reject-driver-host driver=%s bit=%d pos=%d\n",
                        driver->makeName().c_str(), driver_bit, driver_pos);
                }
            }
            if (!driver_fits) {
                if (packDebugEnabled()) {
                    std::fprintf(stderr, "pack-debug     shared-lane no-driver-fit sink_bit=%d driver=%s\n",
                        sink_bit, driver->makeName().c_str());
                }
                all_future_drivers_fit = false;
                break;
            }
        }
        if (all_future_drivers_fit
            && futureStrictOutputSinksFit(tile, sink, sink_type, sink_bit, reserved)) {
            return true;
        }
    }
    return false;
}

bool futureStrictOutputSinksFit(Tile& tile, rtl::Inst& future_inst, ElementType future_type,
                                int future_bit,
                                const std::array<uint16_t, ELEMENT_TYPE_COUNT>& reserved)
{
    // A reserved future producer lane must also fit its own strict mux consumer lane.
    (void) reserved;
    for (rtl::Conn& output : future_inst.conns) {
        if (!output.port_ref.peer || output.port_ref->type != rtl::Port::PORT_OUT
            || output.peer) {
            continue;
        }
        for (auto* sink_ref : rtl::Conn::getSinks(output)) {
            rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            rtl::Inst* sink = sink_conn ? sink_conn->inst_ref.peer : nullptr;
            if (!sink || !sink->cell_ref.peer) {
                continue;
            }
            std::optional<ElementType> sink_type_opt = maybeInstElementType(*sink);
            if (!sink_type_opt) {
                continue;
            }
            ElementType sink_type = *sink_type_opt;
            if (!strictLocalChainInput(future_type, sink_type, sink_conn ? sink_conn->port_ref.peer : nullptr)) {
                continue;
            }
            if (sink->tile.peer) {
                if (!sameAssignedTile(tile, *sink)) {
                    return false;
                }
                int sink_bit = elementBitFromPlacedPos(sink_type, sink->pos);
                if (sink_bit < 0 || !placedStrictPeerUsesLane(tile, future_type, future_bit,
                                                              sink_type, sink_bit, true)) {
                    return false;
                }
                continue;
            }
            if (!hasSharedFreeSinkLane(tile, *sink, future_inst, future_type, future_bit, false)) {
                if (packDebugEnabled()) {
                    std::fprintf(stderr,
                        "pack-debug     shared-lane future-output-no-fit future=%s bit=%d sink=%s\n",
                        future_inst.makeName().c_str(), future_bit, sink->makeName().c_str());
                }
                return false;
            }
        }
    }
    return true;
}

bool placedStrictPeerUsesLane(Tile& tile, ElementType type, int bit, ElementType peer_type,
                              int peer_bit, bool peer_on_right)
{
    // A selected strict-chain element bit must be physically linked to placed peers.
    return (linkedNeighborMask(tile, type, bit, peer_type, peer_on_right) & bit16(peer_bit)) != 0;
}

rtl::Conn* findConn(rtl::Inst& inst, const std::string& port_name, int port_type)
{
    for (auto& conn : inst.conns) {
        if (conn.port_ref.peer && conn.port_ref->name == port_name && conn.port_ref->type == port_type) {
            return &conn;
        }
    }
    return nullptr;
}

rtl::Conn* firstInputConn(rtl::Inst& inst)
{
    for (auto& conn : inst.conns) {
        if (conn.port_ref.peer && conn.port_ref->type == rtl::Port::PORT_IN) {
            return &conn;
        }
    }
    return nullptr;
}

rtl::Conn* firstOutputConn(rtl::Inst& inst)
{
    for (auto& conn : inst.conns) {
        if (conn.port_ref.peer && conn.port_ref->type == rtl::Port::PORT_OUT
            && !conn.peer) {
            return &conn;
        }
    }
    return nullptr;
}

bool outputOnlyDrives(rtl::Inst& driver, rtl::Inst& allowed_sink)
{
    // Sharing a chain output is legal only when the left output is purely tile-local.
    rtl::Conn* output = firstOutputConn(driver);
    if (!output) {
        return true;
    }
    for (auto* sink_ref : rtl::Conn::getSinks(*output)) {
        rtl::Conn* sink = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
        if (!sink || !sink->inst_ref.peer) {
            continue;
        }
        if (sink->inst_ref.peer != &allowed_sink) {
            return false;
        }
    }
    return true;
}

bool hasExternalOutputNet(rtl::Inst& inst)
{
    // A non-void output net means this element may need a fabric-facing local.
    rtl::Conn* output = firstOutputConn(inst);
    if (!output || !output->port_ref.peer) {
        return false;
    }
    rtl::Net* net = findNetByDesignator(inst, output->port_ref->designator);
    return net && !net->designatorIsVoid(output->port_ref->designator);
}

bool connHasExternalNet(rtl::Inst& inst, rtl::Conn& conn)
{
    // A non-void input net means this pin still needs an exclusive fabric local.
    if (!conn.port_ref.peer || conn.port_ref->designator < 0) {
        return false;
    }
    rtl::Net* net = findNetByDesignator(inst, conn.port_ref->designator);
    return net && !net->designatorIsVoid(conn.port_ref->designator);
}

std::string passthroughCellType(ElementType type)
{
    switch (type) {
    case ELEMENT_LUT5: return "LUT5";
    case ELEMENT_LUT1: return "LUT1";
    case ELEMENT_MUXF7: return "MUXF7";
    case ELEMENT_MUXF8: return "MUXF8";
    case ELEMENT_CARRY: return "CARRY4";
    case ELEMENT_FD: return "FDRE";
    default: return "PASSTHROUGH";
    }
}

std::pair<std::string, std::string> passthroughPorts(ElementType type)
{
    switch (type) {
    case ELEMENT_FD: return {"D", "Q"};
    case ELEMENT_CARRY: return {"S0", "O0"};
    case ELEMENT_MUXF7:
    case ELEMENT_MUXF8: return {"I1", "O"};
    default: return {"I0", "O"};
    }
}

std::string_view generatedPassthroughKind(const rtl::Inst* inst)
{
    // Only explicit source/target values identify generated endpoint cells.
    if (!inst || !inst->cell_ref.peer) {
        return {};
    }
    const rtl::Cell* cell = inst->cell_ref.peer;
    auto it = cell->attributes.find("scalepnr_passthrough");
    if (it == cell->attributes.end()
        || (it->second != "source" && it->second != "target")) {
        return {};
    }
    return it->second;
}

bool forcesFabricInput(const rtl::Inst* inst)
{
    // Generated physical endpoints may require fabric routing despite a normally local element pairing.
    const rtl::Cell* cell = inst ? inst->cell_ref.peer : nullptr;
    return cell
        && cell->attributes.contains("scalepnr_force_fabric_input")
        && cell->attributes.at("scalepnr_force_fabric_input") == "1";
}

bool forcesFabricOutput(const rtl::Inst* inst)
{
    // A directly mapped output endpoint must not be replaced by a tile-local passthrough.
    const rtl::Cell* cell = inst ? inst->cell_ref.peer : nullptr;
    return cell
        && cell->attributes.contains("scalepnr_force_fabric_output")
        && cell->attributes.at("scalepnr_force_fabric_output") == "1";
}

bool isSourcePassthroughPort(ElementType type, std::string port)
{
    int bit = extractIndexedPort(port);
    if (type == ELEMENT_FD) {
        return port == "Q";
    }
    if (type == ELEMENT_CARRY) {
        return port == "O" || port == "CO" || (bit >= 0 && (port == "O" || port == "CO"));
    }
    return port == "O" || port == "O5" || port == "O6";
}

bool isTargetPassthroughPort(ElementType type, std::string port)
{
    int bit = extractIndexedPort(port);
    if (type == ELEMENT_FD) {
        return false;
    }
    if (type == ELEMENT_CARRY) {
        return port == "DI" || port == "S" || (bit >= 0 && (port == "DI" || port == "S"));
    }
    if (type == ELEMENT_MUXF7 || type == ELEMENT_MUXF8) {
        return port == "I" || port == "I0" || port == "I1";
    }
    return port == "I" || port.starts_with("I") || port.starts_with("A");
}

rtl::Module* ownerModule(rtl::Inst& inst)
{
    if (inst.parent_ref.peer && inst.parent_ref->cell_ref.peer) {
        return inst.parent_ref->cell_ref->module_ref.peer;
    }
    return parentModule(inst);
}

int nextGeneratedDesignator(rtl::Module& module)
{
    return moduleNetDesignatorIndex(module).next_generated++;
}

rtl::Net* findNetInModuleByDesignator(rtl::Module& module, int designator)
{
    ModuleNetDesignatorIndex& index = moduleNetDesignatorIndex(module);
    auto found = index.net_by_designator.find(designator);
    if (found != index.net_by_designator.end()
        && found->second < module.nets.size()) {
        return &module.nets[found->second];
    }
    return nullptr;
}

rtl::Net* appendGeneratedNet(rtl::Module& module, const std::string& name, int designator, bool void_net)
{
    auto& net = module.nets.emplace_back();
    net.name = void_net ? std::string("void") : name;
    net.designators.push_back(designator);
    if (void_net) {
        net.markDesignatorVoid(designator);
    }
    return &net;
}

Referable<rtl::Cell>* makeGeneratedCell(rtl::Inst& near_inst, ElementType type,
                                        std::string input_port_override = {})
{
    static int generated_cell_index = 0;
    auto* cell = new Referable<rtl::Cell>();
    auto [input_port, output_port] = passthroughPorts(type);
    if (!input_port_override.empty()) {
        input_port = std::move(input_port_override);
    }
    cell->name = std::format("$scalepnr_passthrough_cell${}", generated_cell_index++);
    cell->type = passthroughCellType(type);
    if (near_inst.cell_ref.peer) {
        cell->module_ref.set(near_inst.cell_ref->module_ref.peer);
    }

    rtl::Port in;
    in.name = input_port;
    in.type = rtl::Port::PORT_IN;
    cell->ports.emplace_back(std::move(in));

    rtl::Port out;
    out.name = output_port;
    out.type = rtl::Port::PORT_OUT;
    cell->ports.emplace_back(std::move(out));
    return cell;
}

rtl::Inst* makeGeneratedPassthroughInst(rtl::Inst& near_inst, ElementType type,
                                        std::string input_port = {})
{
    static int generated_inst_index = 0;
    Referable<rtl::Inst>* parent = near_inst.parent_ref.peer;
    PNR_ASSERT(parent, "cannot insert passthrough for '{}' without a parent instance", near_inst.makeName());

    auto& inst = parent->insts.emplace_back();
    inst.cell_ref.set(makeGeneratedCell(near_inst, type, std::move(input_port)));
    inst.parent_ref.set(parent);
    inst.cnt_inputs = 1;
    inst.cnt_outputs = 1;
    inst.pos = -1;
    inst.depth = near_inst.depth;
    inst.height = near_inst.height;
    inst.cell_ref->name = std::format("$scalepnr_passthrough${}", generated_inst_index++);

    for (auto& port : inst.cell_ref->ports) {
        auto& conn = inst.conns.emplace_back();
        conn.port_ref.set(&port);
        conn.inst_ref.set(&inst);
    }
    return &inst;
}

void markVoidNetsBetween(rtl::Inst& driver, rtl::Inst& sink)
{
    // Mark direct same-tile resource-chain nets as internal; routing can skip them.
    std::optional<ElementType> driver_type = maybeInstElementType(driver);
    std::optional<ElementType> sink_type = maybeInstElementType(sink);
    for (auto& conn : sink.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        rtl::Conn* followed = conn.follow();
        if (!followed || followed->inst_ref.peer != &driver || !followed->port_ref.peer
            || followed->port_ref->type != rtl::Port::PORT_OUT) {
            continue;
        }
        // A mux selector is a fabric-facing pin even when its driver happens
        // to share the tile; only modeled strict data arcs may become void.
        if (isMux(sink)
            && (!driver_type || !sink_type
                || !strictLocalChainInput(*driver_type, *sink_type,
                                          conn.port_ref.peer))) {
            continue;
        }
        if (rtl::Net* net = findNetByDesignator(sink, conn.port_ref->designator)) {
            if (!net->designatorIsVoid(conn.port_ref->designator)) {
                fpga::unrouteNetConnection(*net, &driver, &sink,
                    followed->port_ref->makeName(), conn.port_ref->makeName());
                net->markDesignatorVoid(conn.port_ref->designator);
            }
        }
    }
}

void markVoidNetsForTile(Tile& tile)
{
    // Refresh internal-chain net flags after a packed shape changes tile occupancy.
    std::vector<rtl::Inst*> insts = assignedInsts(tile);
    for (rtl::Inst* driver : insts) {
        for (rtl::Inst* sink : insts) {
            if (!driver || !sink || driver == sink) {
                continue;
            }
            if ((isMux(*sink) || isCarry(*sink)) && drivesInput(*driver, *sink)) {
                markVoidNetsBetween(*driver, *sink);
            }
        }
    }
}

rtl::Inst* lutAtBel(Tile& tile, int site, int bel)
{
    // Find the LUT occupying a given BEL index in this tile.
    for (rtl::Inst* inst : assignedInsts(tile)) {
        if (inst && isLut(*inst) && siteIndexFromPlacedPos(inst->pos) == site && belIndexFromPlacedPos(inst->pos) == bel) {
            return inst;
        }
    }
    return nullptr;
}

const Element* elementFor(TileType& type, ElementType element_type, int bit)
{
    for (const Element& element : type.elements) {
        if (element.type == element_type && element.bitmap_pos == bit) {
            return &element;
        }
    }
    return nullptr;
}

rtl::Inst* elementInstAt(Tile& tile, ElementType type, int bit)
{
    // Find an assigned instance that consumes one abstract element bit.
    for (rtl::Inst* inst : assignedInsts(tile)) {
        if (!inst || !inst->cell_ref.peer) {
            continue;
        }
        ElementType inst_type = instElementType(*inst);
        if (type == ELEMENT_LUT1 && inst_type == ELEMENT_LUT5 && isFullLut6(*inst)
            && elementBitFromPlacedPos(inst_type, inst->pos) == bit) {
            return inst;
        }
        if (inst_type == type && elementBitFromPlacedPos(inst_type, inst->pos) == bit) {
            return inst;
        }
    }
    return nullptr;
}

bool tileTypeHasLogicElements(const TileType& type)
{
    return std::any_of(type.elements.begin(), type.elements.end(), [](const Element& element) {
        return element.type < ELEMENT_TYPE_COUNT;
    });
}

void ensureElementState(Tile& tile)
{
    // Mirror TileType element metadata into per-tile free/connected bit arrays.
    if (tile.elements_initialized) {
        return;
    }
    if (!tile.tile_type) {
        return;
    }

    tile.elements_pos = {};
    tile.elements_left = {};
    tile.elements_right = {};
    for (const Element& element : tile.tile_type->elements) {
        tile.elements_pos[element.type] |= bit16(element.bitmap_pos);
        for (int bit = 0; bit < ELEMENT_BITMAP_BITS; ++bit) {
            tile.elements_left[element.type][bit] |= element.left_blockers[bit];
            tile.elements_right[element.type][bit] |= element.right_blockers[bit];
        }
    }
    tile.elements_free = tile.elements_pos;
    for (rtl::Inst* inst : assignedInsts(tile)) {
        if (!inst || !inst->cell_ref.peer) {
            continue;
        }
        ElementType type = instElementType(*inst);
        int bit = elementBitFromPlacedPos(type, inst->pos);
        reserveElementBit(tile, type, bit, inst);
    }
    tile.elements_initialized = true;
    if (packDebugEnabled()) {
        std::fprintf(stderr, "pack-debug tile=%s full=%s type=%s initialized\n",
            tile.makeName().c_str(), tile.full_name.c_str(), tile.tile_type->name.c_str());
        printTypeMasks("pack-debug   pos ", tile.elements_pos);
        printTypeMasks("pack-debug   free", tile.elements_free);
    }
}

enum class BlockerStatus
{
    clear,
    compatible,
    incompatible,
};

BlockerStatus linkedElementStatus(Tile& tile, rtl::Inst* inst, ElementType type, int bit, bool left_side,
                                  std::array<std::array<bool, ELEMENT_BITMAP_BITS>, ELEMENT_TYPE_COUNT>& visited)
{
    // Follow free resource columns until an occupied blocker proves compatibility.
    if (bit < 0 || bit >= ELEMENT_BITMAP_BITS) {
        return BlockerStatus::clear;
    }
    if (visited[type][bit]) {
        return BlockerStatus::clear;
    }
    visited[type][bit] = true;

    int current_column = elementColumn(type);
    const std::array<uint16_t, ELEMENT_BITMAP_BITS>& links = left_side ? tile.elements_left[type] : tile.elements_right[type];

    for (int distance = 1; distance < ELEMENT_TYPE_COUNT; ++distance) {
        int target_column = left_side ? current_column - distance : current_column + distance;
        if (target_column < 0 || target_column >= ELEMENT_TYPE_COUNT) {
            continue;
        }
        BlockerStatus column_status = BlockerStatus::clear;
        for (int type_index = 0; type_index < ELEMENT_TYPE_COUNT; ++type_index) {
            ElementType neighbor_type = static_cast<ElementType>(type_index);
            if (elementColumn(neighbor_type) != target_column) {
                continue;
            }
            for (int neighbor_bit = 0; neighbor_bit < ELEMENT_BITMAP_BITS; ++neighbor_bit) {
                uint16_t mask = links[neighbor_bit];
                if ((mask & bit16(bit)) == 0) {
                    continue;
                }
                const std::array<uint16_t, ELEMENT_BITMAP_BITS>& reciprocal =
                    left_side ? tile.elements_right[neighbor_type] : tile.elements_left[neighbor_type];
                if ((reciprocal[bit] & bit16(neighbor_bit)) == 0) {
                    continue;
                }
                PNR_ASSERT((tile.elements_pos[neighbor_type] & bit16(neighbor_bit)) != 0,
                    "element {} bit {} references missing neighbor {} bit {} in tile type {}",
                    elementTypeName(type), bit, elementTypeName(neighbor_type), neighbor_bit, tile.tile_type->name);
                if ((tile.elements_free[neighbor_type] & bit16(neighbor_bit)) != 0) {
                    BlockerStatus child_status = linkedElementStatus(tile, inst, neighbor_type, neighbor_bit, left_side, visited);
                    if (child_status == BlockerStatus::incompatible) {
                        return BlockerStatus::incompatible;
                    }
                    if (child_status == BlockerStatus::compatible) {
                        column_status = BlockerStatus::compatible;
                    }
                    continue;
                }
                rtl::Inst* neighbor = elementInstAt(tile, neighbor_type, neighbor_bit);
                if (!neighbor) {
                    return BlockerStatus::incompatible;
                }
                if (left_side) {
                    if (!connectedInOrder(*neighbor, *inst)) {
                        if (packDebugEnabled()) {
                            std::fprintf(stderr, "pack debug: left blocker %s bit %d (%s) is not connected to %s bit %d (%s)\n",
                                elementTypeName(neighbor_type), neighbor_bit, neighbor->makeName().c_str(),
                                elementTypeName(type), bit, inst->makeName().c_str());
                        }
                        return BlockerStatus::incompatible;
                    }
                    if (neighbor_type == ELEMENT_LUT1 && type == ELEMENT_CARRY
                        && !outputOnlyDrives(*neighbor, *inst)) {
                        if (packDebugEnabled()) {
                            std::fprintf(stderr, "pack debug: left blocker %s bit %d (%s) has external output users\n",
                                elementTypeName(neighbor_type), neighbor_bit, neighbor->makeName().c_str());
                        }
                        return BlockerStatus::incompatible;
                    }
                }
                else if (!connectedInOrder(*inst, *neighbor)) {
                    if (packDebugEnabled()) {
                        std::fprintf(stderr, "pack debug: %s bit %d (%s) is not connected to right blocker %s bit %d (%s)\n",
                            elementTypeName(type), bit, inst->makeName().c_str(),
                            elementTypeName(neighbor_type), neighbor_bit, neighbor->makeName().c_str());
                    }
                    return BlockerStatus::incompatible;
                }
                else if (type == ELEMENT_LUT1 && neighbor_type == ELEMENT_CARRY
                    && !outputOnlyDrives(*inst, *neighbor)) {
                    if (packDebugEnabled()) {
                        std::fprintf(stderr, "pack debug: %s bit %d (%s) has external output users\n",
                            elementTypeName(type), bit, inst->makeName().c_str());
                    }
                    return BlockerStatus::incompatible;
                }
                column_status = BlockerStatus::compatible;
            }
        }
        if (column_status == BlockerStatus::compatible) {
            return BlockerStatus::compatible;
        }
    }
    return BlockerStatus::clear;
}

NodeMask outputNodesForElement(Tile& tile, ElementType type, int bit, rtl::Inst* inst)
{
    // Resolve output locals for either a real placed cell or generated candidate.
    int pos = inst && inst->pos >= 0 ? inst->pos : placedPosFromElementBit(type, bit);
    if (pos < 0) {
        return {};
    }
    if (inst && inst->cell_ref.peer) {
        rtl::Conn* output = firstOutputConn(*inst);
        if (output && output->port_ref.peer) {
            return tile.getOutputPinNodes(inst->cell_ref->type, output->port_ref->makeName(), pos);
        }
    }
    auto [input_port, output_port] = passthroughPorts(type);
    return tile.getOutputPinNodes(passthroughCellType(type), output_port, pos);
}

bool outputLocalCompatible(Tile& tile, rtl::Inst* inst, ElementType type, int bit)
{
    // Reject external outputs that alias an already occupied output local.
    if (generatedPassthroughKind(inst).empty()) {
        return true;
    }
    NodeMask candidate_nodes = outputNodesForElement(tile, type, bit, inst);
    if (candidate_nodes == NodeMask{}) {
        return true;
    }
    bool candidate_external = inst && hasExternalOutputNet(*inst);
    for (rtl::Inst* owner : assignedInsts(tile)) {
        if (!owner || owner == inst || !owner->cell_ref.peer) {
            continue;
        }
        ElementType owner_type = instElementType(*owner);
        int owner_bit = elementBitFromPlacedPos(owner_type, owner->pos);
        NodeMask owner_nodes = outputNodesForElement(tile, owner_type, owner_bit, owner);
        if ((candidate_nodes & owner_nodes) == NodeMask{}) {
            continue;
        }
        if (connectedInOrder(*owner, *inst) && outputOnlyDrives(*owner, *inst)) {
            continue;
        }
        if (connectedInOrder(*inst, *owner) && outputOnlyDrives(*inst, *owner)) {
            continue;
        }
        if (strictLocalChainReachable(*owner, *inst)
            || strictLocalChainReachable(*inst, *owner)) {
            // A loaded multi-column chain may intentionally reuse one local
            // identity between its first and final packed elements.
            continue;
        }
        if (candidate_external || hasExternalOutputNet(*owner)) {
            if (packDebugEnabled()) {
                std::fprintf(stderr, "pack-debug   reject bit=%d reason=output-alias owner=%s owner_bit=%d\n",
                    bit, owner->makeName().c_str(), owner_bit);
            }
            return false;
        }
    }
    return true;
}

struct InputRouteEndpoint
{
    std::string route_type;
    int local = -1;
};

struct DrivenInputRouteEndpoint
{
    // Preserve the logical driver so shared physical control pins may reuse one signal.
    InputRouteEndpoint endpoint;
    rtl::Conn* driver = nullptr;
};

bool sameInputRouteEndpoint(const InputRouteEndpoint& a, const InputRouteEndpoint& b)
{
    return a.local == b.local
        && a.route_type == b.route_type;
}

void rememberInputRouteEndpoint(std::vector<InputRouteEndpoint>& endpoints, InputRouteEndpoint endpoint)
{
    if (endpoint.local < 0) {
        return;
    }
    auto same = [&](const InputRouteEndpoint& existing) {
        return sameInputRouteEndpoint(existing, endpoint);
    };
    if (std::find_if(endpoints.begin(), endpoints.end(), same) == endpoints.end()) {
        endpoints.push_back(std::move(endpoint));
    }
}

std::vector<int> inputResourceNodesForPin(Tile& tile, const std::string& type, const std::string& port, int pos)
{
    // Resource nodes preserve which concrete route-tile endpoint owns a packed input pin.
    std::vector<int> resources;
    if (!tile.tile_type) {
        return resources;
    }
    if (useResourcePinNameFallback(type)) {
        std::string pin = modeledResourcePinName(tile.tile_type, type, port, pos);
        int site_pos = modeledSitePos(tile.tile_type, pos);
        resources = tile.tile_type->pin_map.resourceNodesForPin(TILE_PIN_INPUT, pin, site_pos);
        return resources;
    }
    int local = tile.getNodeNum(type, port, pos);
    int resource_node = modeledResourceNodeNum(tile.tile_type, type, pos, local);
    if (resource_node >= 0) {
        resources.push_back(resource_node);
    }
    return resources;
}

std::vector<InputRouteEndpoint> inputRouteEndpointsForPin(Tile& tile, const std::string& type,
                                                          const std::string& port, int pos)
{
    // Endpoint identities distinguish the same local number on different adjacent route tiles.
    std::vector<InputRouteEndpoint> endpoints;
    if (!tile.tile_type) {
        return endpoints;
    }
    for (int resource_node : inputResourceNodesForPin(tile, type, port, pos)) {
        NodeMask nodes = tile.tile_type->pin_map.getInputNodes(resource_node);
        nodes.for_each_set_bit([&](int local) {
            TilePinEndpointNameKey key{static_cast<uint8_t>(TILE_PIN_INPUT), resource_node, local};
            auto route_it = tile.tile_type->pin_map.endpoint_route_refs.find(key);
            if (route_it == tile.tile_type->pin_map.endpoint_route_refs.end()) {
                rememberInputRouteEndpoint(endpoints, InputRouteEndpoint{"", local});
                return false;
            }
            for (const TilePinEndpointRouteRef& ref : route_it->second) {
                rememberInputRouteEndpoint(endpoints, InputRouteEndpoint{ref.route_type, local});
            }
            return false;
        });
    }
    if (endpoints.empty()) {
        NodeMask nodes = tile.getPinNodes(type, port, pos);
        nodes.for_each_set_bit([&](int local) {
            rememberInputRouteEndpoint(endpoints, InputRouteEndpoint{"", local});
            return false;
        });
    }
    return endpoints;
}

std::vector<InputRouteEndpoint> inputRouteEndpointsForInstAt(Tile& tile, rtl::Inst& inst, int pos, bool external_only)
{
    // Collect route-endpoint identities for all routed input pins of one candidate placement.
    std::vector<InputRouteEndpoint> endpoints;
    if (!inst.cell_ref.peer || pos < 0) {
        return endpoints;
    }
    for (rtl::Conn& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        if (external_only && !connHasExternalNet(inst, conn)) {
            continue;
        }
        for (InputRouteEndpoint endpoint : inputRouteEndpointsForPin(tile, inst.cell_ref->type,
                 conn.port_ref->makeName(), pos)) {
            rememberInputRouteEndpoint(endpoints, std::move(endpoint));
        }
    }
    return endpoints;
}

std::vector<DrivenInputRouteEndpoint> drivenInputRouteEndpointsForInstAt(
    Tile& tile, rtl::Inst& inst, int pos, bool external_only)
{
    // Associate every concrete input endpoint with the signal that drives it.
    std::vector<DrivenInputRouteEndpoint> endpoints;
    if (!inst.cell_ref.peer || pos < 0) {
        return endpoints;
    }
    for (rtl::Conn& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        if (external_only && !connHasExternalNet(inst, conn)) {
            continue;
        }
        rtl::Conn* driver = conn.follow();
        for (InputRouteEndpoint endpoint : inputRouteEndpointsForPin(
                 tile, inst.cell_ref->type, conn.port_ref->makeName(), pos)) {
            endpoints.push_back(DrivenInputRouteEndpoint{std::move(endpoint), driver});
        }
    }
    return endpoints;
}

bool independentInputEndpointConflict(
    const std::vector<DrivenInputRouteEndpoint>& candidate,
    const std::vector<DrivenInputRouteEndpoint>& owner)
{
    // One endpoint may feed multiple packed cells only when they share its driver.
    for (const DrivenInputRouteEndpoint& a : candidate) {
        for (const DrivenInputRouteEndpoint& b : owner) {
            if (sameInputRouteEndpoint(a.endpoint, b.endpoint) && a.driver != b.driver) {
                return true;
            }
        }
    }
    return false;
}

NodeMask inputNodesForInstAt(Tile& tile, rtl::Inst& inst, int pos, bool external_only)
{
    // Collect local input nodes used by routed input pins of one placed instance.
    if (!inst.cell_ref.peer || pos < 0) {
        return {};
    }
    NodeMask nodes{};
    for (rtl::Conn& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        if (external_only && !connHasExternalNet(inst, conn)) {
            continue;
        }
        nodes |= tile.getPinNodes(inst.cell_ref->type, conn.port_ref->makeName(), pos);
    }
    return nodes;
}

NodeMask inputNodesForInst(Tile& tile, rtl::Inst& inst, bool external_only)
{
    return inputNodesForInstAt(tile, inst, inst.pos, external_only);
}

NodeMask inputNodesForRouteTileAt(Tile& resource_tile, const Tile& route_tile,
                                  rtl::Inst& inst, rtl::Conn& conn, int pos)
{
    // Match placement legality to routing's exact resource-to-crossbar endpoint selection.
    if (!inst.cell_ref.peer || !conn.port_ref.peer || !route_tile.cb_type) {
        return {};
    }
    Coord route_delta = route_tile.coord - resource_tile.coord;
    NodeMask nodes = resource_tile.getPinNodesForRouteType(inst.cell_ref->type,
        conn.port_ref->makeName(), pos, TILE_PIN_INPUT, route_tile.cb_type->name, route_delta);
    if (nodes == NodeMask{} && resource_tile.tile_type
        && resource_tile.tile_type->pin_map.endpoint_route_refs.empty()) {
        nodes = resource_tile.getPinNodes(inst.cell_ref->type, conn.port_ref->makeName(), pos);
    }
    return nodes;
}

NodeMask inputNodesForElement(Tile& tile, ElementType type, int bit, rtl::Inst* inst)
{
    // Resolve input locals for either a real placed cell or generated candidate.
    int pos = placedPosFromElementBit(type, bit);
    if (pos < 0) {
        return {};
    }
    if (inst && inst->cell_ref.peer) {
        return inputNodesForInstAt(tile, *inst, pos, true);
    }
    auto [input_port, output_port] = passthroughPorts(type);
    return tile.getPinNodes(passthroughCellType(type), input_port, pos);
}

NodeMask mandatoryInputJoints(const CBType& cb_type, int local, NodeMask incoming_dsts = {})
{
    // Intersect joints used by every physically reachable dst-to-local path.
    const_cast<CBType&>(cb_type).ensureDerivedMasks();
    NodeMask mandatory{};
    bool first_path = true;
    NodeMask direct_to_local = cb_type.local_reachable_joints[local].joint;
    auto include_path = [&](NodeMask path_joints) {
        mandatory = first_path ? path_joints : (mandatory & path_joints);
        first_path = false;
    };
    NodeMask reachable_dsts = cb_type.dsts_reaching_local[local].jump;
    if (incoming_dsts != NodeMask{}) {
        reachable_dsts &= incoming_dsts;
    }
    reachable_dsts.for_each_set_bit([&](int dst) {
        if ((cb_type.dst_local[dst].local & (NodeMask{0,1} << local)) != NodeMask{}) {
            include_path({});
        }
        NodeMask first_joints = cb_type.dst_joint[dst].joint;
        (first_joints & direct_to_local).for_each_set_bit([&](int joint) {
            include_path(NodeMask{0,1} << joint);
            return false;
        });
        first_joints.for_each_set_bit([&](int first_joint) {
            (cb_type.joint_joint[first_joint].joint & direct_to_local).for_each_set_bit([&](int second_joint) {
                include_path((NodeMask{0,1} << first_joint) | (NodeMask{0,1} << second_joint));
                return false;
            });
            return false;
        });
        return mandatory == NodeMask{} && !first_path;
    });
    return first_path ? NodeMask{} : mandatory;
}

NodeMask mandatoryInputJointsForTile(Tile& route_tile, int local)
{
    // Crossbar topology and incoming-node availability are immutable after
    // grid construction, so each local's mandatory joints need one derivation.
    auto found = route_tile.mandatory_input_joints.find(local);
    if (found != route_tile.mandatory_input_joints.end()) {
        return found->second;
    }
    NodeMask joints = route_tile.cb_type
        ? mandatoryInputJoints(*route_tile.cb_type, local, route_tile.incoming_dst_nodes)
        : NodeMask{};
    route_tile.mandatory_input_joints.emplace(local, joints);
    return joints;
}

const std::vector<Tile*>& attachedResourceTiles(Tile& resource_tile)
{
    // Collect resource tiles attached to the same physical route crossbar.
    if (!resource_tile.attached_resource_tiles.empty()) {
        return resource_tile.attached_resource_tiles;
    }
    Device& device = Device::current();
    Coord canonical = resource_tile.cb_coord.x >= 0 && resource_tile.cb_coord.y >= 0
        ? resource_tile.cb_coord : resource_tile.coord;
    constexpr int attached_resource_radius = 6;
    std::vector<Tile*>& tiles = resource_tile.attached_resource_tiles;
    for (int dy = -attached_resource_radius; dy <= attached_resource_radius; ++dy) {
        for (int dx = -attached_resource_radius; dx <= attached_resource_radius; ++dx) {
            Tile* candidate = device.getTile(canonical.x + dx, canonical.y + dy);
            if (!candidate) {
                continue;
            }
            Coord candidate_canonical = candidate->cb_coord.x >= 0 && candidate->cb_coord.y >= 0
                ? candidate->cb_coord : candidate->coord;
            if (candidate_canonical.x == canonical.x && candidate_canonical.y == canonical.y) {
                tiles.push_back(candidate);
            }
        }
    }
    if (std::find(tiles.begin(), tiles.end(), &resource_tile) == tiles.end()) {
        tiles.push_back(&resource_tile);
    }
    return tiles;
}

void ensureInputJointReservations(Tile& route_tile)
{
    // Cache exact input locals and mandatory joints; placement changes invalidate both lists.
    if (route_tile.input_joint_reservations_initialized) {
        return;
    }
    route_tile.input_local_reservations.clear();
    route_tile.input_joint_reservations.clear();
    for (Tile* owner_tile : attachedResourceTiles(route_tile)) {
        for (rtl::Inst* owner : assignedInsts(*owner_tile)) {
            if (!owner || !owner->cell_ref.peer) {
                continue;
            }
            for (rtl::Conn& owner_conn : owner->conns) {
                if (!owner_conn.port_ref.peer || owner_conn.port_ref->type != rtl::Port::PORT_IN
                    || !connHasExternalNet(*owner, owner_conn)) {
                    continue;
                }
                NodeMask owner_joints;
                NodeMask owner_locals = inputNodesForRouteTileAt(
                    *owner_tile, route_tile, *owner, owner_conn, owner->pos);
                rtl::Conn* owner_driver = owner_conn.follow();
                owner_locals.for_each_set_bit([&](int owner_local) {
                    auto [it, inserted] = route_tile.input_local_reservations.emplace(
                        owner_local, owner_driver);
                    if (!inserted && it->second != owner_driver) {
                        it->second = nullptr;
                    }
                    return false;
                });
                owner_locals.for_each_set_bit([&](int owner_local) {
                    owner_joints |= mandatoryInputJointsForTile(route_tile, owner_local);
                    return false;
                });
                if (owner_joints != NodeMask{}) {
                    route_tile.input_joint_reservations.emplace_back(
                        owner_conn.follow(), owner_joints);
                }
            }
        }
    }
    route_tile.input_joint_reservations_initialized = true;
}

bool inputLocalReservedByDriverImpl(Tile& route_tile, int local,
                                    rtl::Conn* driver)
{
    // Build the placement-owned endpoint index before comparing canonical drivers.
    if (local < 0 || !driver) {
        return false;
    }
    ensureInputJointReservations(route_tile);
    auto owner = route_tile.input_local_reservations.find(local);
    return owner != route_tile.input_local_reservations.end()
        && owner->second == driver;
}

bool inputEndpointCompatible(Tile& tile, rtl::Inst& inst, int pos)
{
    // A concrete route-tile local may serve several packed cells only for one signal.
    Device& device = Device::current();
    Tile* route_tile = device.routeTile(tile);
    if (!route_tile || !route_tile->cb_type || !inst.cell_ref.peer) {
        return true;
    }
    ensureInputJointReservations(*route_tile);
    for (rtl::Conn& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN
            || !connHasExternalNet(inst, conn)) {
            continue;
        }
        rtl::Conn* candidate_driver = conn.follow();
        NodeMask candidate_locals = inputNodesForRouteTileAt(
            tile, *route_tile, inst, conn, pos);
        bool conflict = false;
        candidate_locals.for_each_set_bit([&](int candidate_local) {
            auto owner = route_tile->input_local_reservations.find(candidate_local);
            if (owner != route_tile->input_local_reservations.end()
                && owner->second != candidate_driver) {
                if (packDebugEnabled()) {
                    std::fprintf(stderr,
                        "pack-debug   input-endpoint-conflict port=%s pos=%d route_tile=(%d,%d) "
                        "local=%d candidate_driver=%s owner_driver=%s\n",
                        conn.port_ref->name.c_str(), pos, route_tile->coord.x, route_tile->coord.y,
                        candidate_local,
                        candidate_driver ? candidate_driver->makeName().c_str() : "<none>",
                        owner->second ? owner->second->makeName().c_str() : "<multiple>");
                }
                conflict = true;
                return true;
            }
            return false;
        });
        if (conflict) {
            return false;
        }
    }
    return true;
}

bool inputJointCompatible(Tile& tile, rtl::Inst& inst, int pos)
{
    // Different external signals cannot consume the same joint when every route to their pin requires it.
    if (!tile.cb_type || !inst.cell_ref.peer) {
        return true;
    }
    Device& device = Device::current();
    Tile* route_tile = device.routeTile(tile);
    if (!route_tile || !route_tile->cb_type) {
        return true;
    }
    NodeMask incoming_dsts = route_tile ? route_tile->incoming_dst_nodes : NodeMask{};
    Tile* reservation_tile = route_tile ? route_tile : &tile;
    for (rtl::Conn& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN
            || !connHasExternalNet(inst, conn)) {
            continue;
        }
        rtl::Conn* candidate_driver = conn.follow();
        NodeMask candidate_locals = inputNodesForRouteTileAt(tile, *route_tile, inst, conn, pos);
        bool has_compatible_local = false;
        candidate_locals.for_each_set_bit([&](int candidate_local) {
            NodeMask candidate_joints = mandatoryInputJointsForTile(*route_tile, candidate_local);
            if (packDebugEnabled()) {
                std::fprintf(stderr,
                    "pack-debug   input-joints inst=%s local=%d incoming=%s mandatory=%s\n",
                    inst.makeName().c_str(), candidate_local, incoming_dsts.str().c_str(),
                    candidate_joints.str().c_str());
            }
            bool local_compatible = true;
            if (candidate_joints != NodeMask{}) {
                ensureInputJointReservations(*reservation_tile);
                for (const auto& [driver, joints] : reservation_tile->input_joint_reservations) {
                    if (driver != candidate_driver
                        && (candidate_joints & joints) != NodeMask{}) {
                        local_compatible = false;
                        break;
                    }
                }
            }
            if (local_compatible) {
                has_compatible_local = true;
                return true;
            }
            return false;
        });
        if (candidate_locals != NodeMask{} && !has_compatible_local) {
            return false;
        }
    }
    return true;
}

bool generatedPassthroughInputNeedsFabric(rtl::Inst& inst)
{
    // Source passthrough inputs are void tile-local; target passthrough inputs are routed.
    std::string_view kind = generatedPassthroughKind(&inst);
    if (kind.empty()) {
        return false;
    }
    if (kind == "target") {
        return true;
    }
    if (kind == "source") {
        return false;
    }
    rtl::Conn* input = firstInputConn(inst);
    return input && connHasExternalNet(inst, *input);
}

bool inputLocalCompatible(Tile& tile, rtl::Inst* inst, ElementType type, int bit)
{
    // Reject routed inputs that alias another cell's routed input local.
    if (!inst || !inst->cell_ref.peer) {
        return true;
    }
    if (!tile.tile_type || tile.tile_type->pin_map.input_nodes.empty()) {
        return true;
    }
    if (!generatedPassthroughKind(inst).empty()
        && !generatedPassthroughInputNeedsFabric(*inst)) {
        return true;
    }
    NodeMask candidate_nodes = inputNodesForElement(tile, type, bit, inst);
    if (candidate_nodes == NodeMask{}) {
        return true;
    }
    int candidate_pos = placedPosFromElementBit(type, bit);
    if (!inputEndpointCompatible(tile, *inst, candidate_pos)) {
        if (packDebugEnabled()) {
            std::fprintf(stderr, "pack-debug   reject bit=%d reason=input-endpoint-conflict inst=%s\n",
                bit, inst->makeName().c_str());
        }
        return false;
    }
    if (!enforce_pack_route_capacity) {
        return true;
    }
    std::vector<InputRouteEndpoint> candidate_endpoints;
    std::vector<DrivenInputRouteEndpoint> candidate_driven_endpoints;
    if (inst && inst->cell_ref.peer) {
        candidate_endpoints = inputRouteEndpointsForInstAt(tile, *inst, candidate_pos, true);
        candidate_driven_endpoints = drivenInputRouteEndpointsForInstAt(
            tile, *inst, candidate_pos, true);
    }
    for (rtl::Inst* owner : assignedInsts(tile)) {
        if (!owner || owner == inst || !owner->cell_ref.peer) {
            continue;
        }
        NodeMask owner_nodes = inputNodesForInst(tile, *owner, true);
        bool endpoint_conflict = (candidate_nodes & owner_nodes) != NodeMask{};
        if (!candidate_endpoints.empty()) {
            endpoint_conflict = false;
            std::vector<InputRouteEndpoint> owner_endpoints =
                inputRouteEndpointsForInstAt(tile, *owner, owner->pos, true);
            if (!owner_endpoints.empty()) {
                std::vector<DrivenInputRouteEndpoint> owner_driven_endpoints =
                    drivenInputRouteEndpointsForInstAt(tile, *owner, owner->pos, true);
                endpoint_conflict = independentInputEndpointConflict(
                    candidate_driven_endpoints, owner_driven_endpoints);
            }
        }
        if (!endpoint_conflict) {
            continue;
        }
        if (strictLocalChainReachable(*owner, *inst) || strictLocalChainReachable(*inst, *owner)) {
            continue;
        }
        if (packDebugEnabled()) {
            std::fprintf(stderr, "pack-debug   reject bit=%d reason=input-alias inst=%s owner=%s nodes=%s\n",
                bit, inst->makeName().c_str(), owner->makeName().c_str(),
                (candidate_nodes & owner_nodes).str().c_str());
        }
        return false;
    }
    if (!inputJointCompatible(tile, *inst, candidate_pos)) {
        if (packDebugEnabled()) {
            std::fprintf(stderr, "pack-debug   reject bit=%d reason=input-joint-conflict inst=%s\n",
                bit, inst->makeName().c_str());
        }
        return false;
    }
    return true;
}

bool neighborsCompatible(Tile& tile, rtl::Inst* inst, ElementType type, int bit)
{
    // Occupied blockers anywhere along a connected resource chain must match the netlist.
    const Element* element = elementFor(*tile.tile_type, type, bit);
    if (!element) {
        return false;
    }
    if (inst && inst->cell_ref.peer) {
        bool target_passthrough_input_is_fabric = generatedPassthroughKind(inst) == "target"
            || forcesFabricInput(inst);
        for (auto& conn : inst->conns) {
            if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
                continue;
            }
            rtl::Conn* driver_conn = conn.follow();
            rtl::Inst* driver = driver_conn ? driver_conn->inst_ref.peer : nullptr;
            if (!driver || !driver->cell_ref.peer) {
                continue;
            }
            std::optional<ElementType> driver_type_opt = maybeInstElementType(*driver);
            if (!driver_type_opt) {
                continue;
            }
            ElementType driver_type = *driver_type_opt;
            // Moving packs the real chain before rehoming a deferred target
            // endpoint, so its still-unplaced output is temporarily external.
            bool deferred_target_input = !driver->tile.peer
                && generatedPassthroughKind(driver) == "target";
            if (deferred_target_input) {
                // Moving packs the real chain before rehoming this endpoint;
                // reserve the exact free predecessor lane selected by its port.
                uint16_t available = linkedNeighborMask(
                    tile, type, bit, driver_type, false) & tile.elements_free[driver_type];
                bool has_lane = false;
                while (available) {
                    int driver_bit = std::countr_zero(static_cast<unsigned>(available));
                    available &= static_cast<uint16_t>(available - 1);
                    if (strictLocalChainLaneMatches(
                            driver_type, driver_bit, type, bit, conn.port_ref.peer)) {
                        has_lane = true;
                        break;
                    }
                }
                if (!has_lane) {
                    if (packDebugEnabled()) {
                        std::fprintf(stderr,
                            "pack-debug   reject bit=%d reason=deferred-target-no-lane inst=%s driver=%s\n",
                            bit, inst->makeName().c_str(), driver->makeName().c_str());
                    }
                    return false;
                }
                continue;
            }
            if (target_passthrough_input_is_fabric) {
                continue;
            }
            if (strictLocalChainInput(driver_type, type, conn.port_ref.peer)) {
                if (driver->tile.peer && !sameAssignedTile(tile, *driver)) {
                    if (packDebugEnabled()) {
                        std::fprintf(stderr, "pack-debug   reject bit=%d reason=chain-driver-other-tile inst=%s driver=%s\n",
                            bit, inst->makeName().c_str(), driver->makeName().c_str());
                    }
                    return false;
                }
                if (driver->tile.peer) {
                    int driver_bit = elementBitFromPlacedPos(driver_type, driver->pos);
                    if (driver_bit < 0
                        || !placedStrictPeerUsesLane(tile, type, bit, driver_type, driver_bit, false)
                        || !strictLocalChainLaneMatches(driver_type, driver_bit, type, bit, conn.port_ref.peer)) {
                        if (packDebugEnabled()) {
                            std::fprintf(stderr, "pack-debug   reject bit=%d reason=chain-driver-lane inst=%s driver=%s driver_bit=%d\n",
                                bit, inst->makeName().c_str(), driver->makeName().c_str(), driver_bit);
                        }
                        return false;
                    }
                }
                if (!driver->tile.peer) {
                    if (packDebugEnabled()) {
                        std::fprintf(stderr, "pack-debug   reject bit=%d reason=chain-driver-unplaced inst=%s driver=%s driver_cell=%s driver_ptr=%p\n",
                            bit,
                            inst->makeName(std::numeric_limits<size_t>::max()).c_str(),
                            driver->makeName(std::numeric_limits<size_t>::max()).c_str(),
                            driver->cell_ref.peer ? driver->cell_ref->type.c_str() : "", static_cast<void*>(driver));
                    }
                    return false;
                }
            }
        }
        for (auto& conn : inst->conns) {
            if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_OUT
                || conn.peer) {
                continue;
            }
            for (auto* sink_ref : rtl::Conn::getSinks(conn)) {
                rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
                rtl::Inst* sink = sink_conn ? sink_conn->inst_ref.peer : nullptr;
                if (!sink || !sink->cell_ref.peer) {
                    continue;
                }
                std::optional<ElementType> sink_type_opt = maybeInstElementType(*sink);
                if (!sink_type_opt) {
                    continue;
                }
                ElementType sink_type = *sink_type_opt;
                if (forcesFabricInput(sink)) {
                    continue;
                }
                if (strictLocalChainInput(type, sink_type, sink_conn ? sink_conn->port_ref.peer : nullptr)) {
                    if (sink->tile.peer && !sameAssignedTile(tile, *sink)) {
                        if (packDebugEnabled()) {
                            std::fprintf(stderr, "pack-debug   reject bit=%d reason=chain-sink-other-tile inst=%s sink=%s\n",
                                bit, inst->makeName().c_str(), sink->makeName().c_str());
                        }
                        return false;
                    }
                    if (sink->tile.peer) {
                        int sink_bit = elementBitFromPlacedPos(sink_type, sink->pos);
                        if (sink_bit < 0
                            || !placedStrictPeerUsesLane(tile, type, bit, sink_type, sink_bit, true)
                            || !strictLocalChainLaneMatches(type, bit, sink_type, sink_bit,
                                                           sink_conn ? sink_conn->port_ref.peer : nullptr)) {
                            if (packDebugEnabled()) {
                                std::fprintf(stderr, "pack-debug   reject bit=%d reason=chain-sink-lane inst=%s sink=%s sink_bit=%d\n",
                                    bit, inst->makeName().c_str(), sink->makeName().c_str(), sink_bit);
                            }
                            return false;
                        }
                    }
                    if (!sink->tile.peer && strictSinkHasPlacedDriverOutside(tile, *sink, *inst)) {
                        if (packDebugEnabled()) {
                            std::fprintf(stderr, "pack-debug   reject bit=%d reason=chain-sink-sibling-driver-other-tile inst=%s sink=%s\n",
                                bit, inst->makeName().c_str(), sink->makeName().c_str());
                        }
                        return false;
                    }
                    if (!sink->tile.peer && !hasSharedFreeSinkLane(tile, *sink, *inst, type, bit)) {
                        if (packDebugEnabled()) {
                            std::fprintf(stderr, "pack-debug   reject bit=%d reason=chain-sink-no-shared-lane inst=%s sink=%s\n",
                                bit, inst->makeName().c_str(), sink->makeName().c_str());
                        }
                        return false;
                    }
                }
            }
        }
    }
    if (!outputLocalCompatible(tile, inst, type, bit)) {
        return false;
    }
    if (!inputLocalCompatible(tile, inst, type, bit)) {
        return false;
    }

    if (isLutElement(type)) {
        ElementType paired_type = type == ELEMENT_LUT1 ? ELEMENT_LUT5 : ELEMENT_LUT1;
        if ((tile.elements_pos[paired_type] & bit16(bit)) != 0
            && (tile.elements_free[paired_type] & bit16(bit)) == 0) {
            rtl::Inst* paired = elementInstAt(tile, paired_type, bit);
            if (!paired) {
                return false;
            }
            bool inst_passthrough = !generatedPassthroughKind(inst).empty();
            bool paired_passthrough = !generatedPassthroughKind(paired).empty();
            if ((inst_passthrough || paired_passthrough)
                && !connectedInOrder(*paired, *inst)
                && !connectedInOrder(*inst, *paired)) {
                if (packDebugEnabled()) {
                    std::fprintf(stderr, "pack-debug   reject bit=%d reason=passthrough-lut-overlay inst=%s paired=%s\n",
                        bit, inst ? inst->makeName().c_str() : "", paired->makeName().c_str());
                }
                return false;
            }
            if (isFullLut6(*inst) || isFullLut6(*paired)) {
                if (packDebugEnabled()) {
                    std::fprintf(stderr, "pack-debug   reject bit=%d reason=lut-overlay inst=%s paired=%s\n",
                        bit, inst ? inst->makeName().c_str() : "",
                        paired ? paired->makeName().c_str() : "");
                }
                return false;
            }
        }
    }

    std::array<std::array<bool, ELEMENT_BITMAP_BITS>, ELEMENT_TYPE_COUNT> left_visited{};
    std::array<std::array<bool, ELEMENT_BITMAP_BITS>, ELEMENT_TYPE_COUNT> right_visited{};

    return linkedElementStatus(tile, inst, type, bit, true, left_visited) != BlockerStatus::incompatible
        && linkedElementStatus(tile, inst, type, bit, false, right_visited) != BlockerStatus::incompatible;
}

bool tryElementPlacement(Tile& tile, rtl::Inst* inst, ElementType type, int& pos)
{
    // Pick one free element bit whose occupied neighbors are netlist-compatible.
    PackDebugScope debug_scope(inst);
    ensureElementState(tile);
    if (!tile.tile_type || !tile.elements_initialized) {
        return false;
    }
    uint16_t free = tile.elements_free[type];
    if (packDebugEnabled()) {
        std::fprintf(stderr, "pack-debug try inst=%s cell=%s element=%s tile=%s full=%s type=%s free=0x%04x\n",
            inst->makeName().c_str(), inst->cell_ref.peer ? inst->cell_ref->type.c_str() : "",
            elementTypeName(type), tile.makeName().c_str(), tile.full_name.c_str(), tile.tile_type->name.c_str(), free);
    }
    while (free) {
        int bit = std::countr_zero(static_cast<unsigned>(free));
        free &= static_cast<uint16_t>(free - 1);
        int candidate_pos = placedPosFromElementBit(type, bit);
        if (candidate_pos < 0 || !canHost(tile, inst, candidate_pos)) {
            if (packDebugEnabled()) {
                std::fprintf(stderr, "pack-debug   reject bit=%d pos=%d reason=host\n", bit, candidate_pos);
            }
            continue;
        }
        if (!neighborsCompatible(tile, inst, type, bit)) {
            if (packDebugEnabled()) {
                std::fprintf(stderr, "pack-debug   reject bit=%d pos=%d reason=chain\n", bit, candidate_pos);
            }
            continue;
        }
        if (packDebugEnabled()) {
            std::fprintf(stderr, "pack-debug   accept bit=%d pos=%d\n", bit, candidate_pos);
        }
        pos = candidate_pos;
        return true;
    }
    if (packDebugEnabled()) {
        std::fprintf(stderr, "pack-debug   failed inst=%s element=%s tile=%s\n",
            inst->makeName().c_str(), elementTypeName(type), tile.makeName().c_str());
    }
    return false;
}

bool elementPlacementAtLegal(Tile& tile, rtl::Inst* inst,
                             ElementType type, int pos)
{
    ensureElementState(tile);
    int bit = elementBitFromPlacedPos(type, pos);
    return tile.tile_type && tile.elements_initialized
        && bit >= 0 && bit < ELEMENT_BITMAP_BITS
        && (tile.elements_pos[type] & bit16(bit)) != 0
        && (tile.elements_free[type] & bit16(bit)) != 0
        && canHost(tile, inst, pos)
        && neighborsCompatible(tile, inst, type, bit);
}

bool placeGeneratedAtElement(Tile& tile, rtl::Inst& inst, ElementType type, int bit)
{
    // Commit a generated passthrough into the exact linked element position.
    PackDebugScope debug_scope(&inst);
    ensureElementState(tile);
    if (bit < 0 || bit >= ELEMENT_BITMAP_BITS || (tile.elements_free[type] & bit16(bit)) == 0) {
        return false;
    }
    int pos = placedPosFromElementBit(type, bit);
    bool host_ok = pos >= 0 && canHost(tile, &inst, pos);
    bool neighbors_ok = host_ok && neighborsCompatible(tile, &inst, type, bit);
    if (!host_ok || !neighbors_ok) {
        if (std::getenv("SCALEPNR_VCC_DEBUG")
            && generatedPassthroughKind(&inst) == "target") {
            PNR_LOG1("FPGA",
                "generated target placement rejected: inst='{}' tile=({},{}) type={} bit={} pos={} host_ok={} neighbors_ok={} free=0x{:04x}",
                inst.makeName(), tile.coord.x, tile.coord.y, elementTypeName(type),
                bit, pos, host_ok, neighbors_ok, tile.elements_free[type]);
        }
        return false;
    }
    inst.pos = pos;
    inst.coord = tile.coord;
    tile.assign(&inst);
    reserveElementBit(tile, type, bit, &inst);
    tile.elements_initialized = true;
    return true;
}

struct NeighborElement
{
    ElementType type = ELEMENT_LUT5;
    int bit = -1;
    std::string input_port;
};

std::optional<NeighborElement> firstFreeNeighbor(Tile& tile, ElementType type, int bit, bool right_side,
                                                 std::string_view current_input_port = {},
                                                 std::optional<ElementType> required_type = std::nullopt,
                                                 bool require_free = true)
{
    // Use copied Tile element connectivity to find a free adjacent chain resource.
    ensureElementState(tile);
    const auto& links = right_side ? tile.elements_right[type] : tile.elements_left[type];
    int current_column = elementColumn(type);
    for (int distance = 1; distance < ELEMENT_TYPE_COUNT; ++distance) {
        int target_column = right_side ? current_column + distance : current_column - distance;
        if (target_column < 0 || target_column >= ELEMENT_TYPE_COUNT) {
            continue;
        }
        for (int type_index = 0; type_index < ELEMENT_TYPE_COUNT; ++type_index) {
            ElementType neighbor_type = static_cast<ElementType>(type_index);
            if (required_type && neighbor_type != *required_type) {
                continue;
            }
            if (elementColumn(neighbor_type) != target_column) {
                continue;
            }
            const auto& reciprocal = right_side
                ? tile.elements_left[neighbor_type] : tile.elements_right[neighbor_type];
            for (int neighbor_bit = 0; neighbor_bit < ELEMENT_BITMAP_BITS; ++neighbor_bit) {
                if ((links[neighbor_bit] & bit16(bit)) == 0) {
                    continue;
                }
                // Equal bit numbers in different element columns are not links
                // unless the candidate type records the reciprocal connection.
                if ((reciprocal[bit] & bit16(neighbor_bit)) == 0) {
                    continue;
                }
                std::string sink_port(current_input_port);
                if (right_side) {
                    if (neighbor_type == ELEMENT_MUXF7 || neighbor_type == ELEMENT_MUXF8) {
                        if (sink_port.empty()) {
                            for (std::string_view candidate_port : {std::string_view{"I0"}, std::string_view{"I1"}}) {
                                if (strictLocalChainLaneMatches(
                                        type, bit, neighbor_type, neighbor_bit, candidate_port)) {
                                    sink_port = candidate_port;
                                    break;
                                }
                            }
                        }
                        if (sink_port.empty()) {
                            continue;
                        }
                    }
                    else {
                        sink_port = passthroughPorts(neighbor_type).first;
                    }
                }
                if (right_side
                    ? !strictLocalChainLaneMatches(type, bit, neighbor_type, neighbor_bit, sink_port)
                    : !strictLocalChainLaneMatches(neighbor_type, neighbor_bit, type, bit, sink_port)) {
                    continue;
                }
                if (right_side && neighbor_type == ELEMENT_FD) {
                    continue;
                }
                // Distributed endpoints retain a typed predecessor even when
                // Moving must relocate the packed chain before it becomes free.
                if (!require_free) {
                    return NeighborElement{neighbor_type, neighbor_bit,
                                           std::move(sink_port)};
                }
                if ((tile.elements_free[neighbor_type] & bit16(neighbor_bit)) != 0) {
                    int pos = placedPosFromElementBit(neighbor_type, neighbor_bit);
                    auto [input_port, output_port] = passthroughPorts(neighbor_type);
                    NodeMask route_nodes = right_side
                        ? tile.getOutputPinNodes(passthroughCellType(neighbor_type), output_port, pos)
                        : tile.getPinNodes(passthroughCellType(neighbor_type), input_port, pos);
                    if (route_nodes != NodeMask{} && (route_nodes & tile.pin_state.leased_nodes) != NodeMask{}) {
                        continue;
                    }
                    return NeighborElement{neighbor_type, neighbor_bit, std::move(sink_port)};
                }
            }
        }
    }
    return std::nullopt;
}

bool hasNeighbor(Tile& tile, ElementType type, int bit, bool right_side)
{
    ensureElementState(tile);
    const auto& links = right_side ? tile.elements_right[type] : tile.elements_left[type];
    for (int neighbor_bit = 0; neighbor_bit < ELEMENT_BITMAP_BITS; ++neighbor_bit) {
        if ((links[neighbor_bit] & bit16(bit)) != 0) {
            return true;
        }
    }
    return false;
}

void invalidateInputJointReservations(rtl::Conn& input)
{
    // Rewired packed inputs change the driver-keyed mandatory-joint cache.
    if (!input.inst_ref.peer || !input.inst_ref->tile.peer) {
        return;
    }
    Tile* route_tile = Device::current().routeTile(*input.inst_ref->tile);
    if (route_tile) {
        route_tile->input_local_reservations.clear();
        route_tile->input_joint_reservations.clear();
        route_tile->input_joint_reservations_initialized = false;
    }
}

void connectConns(rtl::Conn& input, rtl::Conn& output, int designator)
{
    // Connect the endpoints and invalidate routing reservations for this sink.
    input.port_ref->designator = designator;
    output.port_ref->designator = designator;
    input.set(&rtl::Conn::fromBase(output));
    invalidateInputJointReservations(input);
}

void refreshPassthroughVoidNets(Tile& tile)
{
    markVoidNetsForTile(tile);
}

bool ensureSourcePassthrough(rtl::Inst*& from, std::string& from_port, rtl::Net*& net,
                             bool allow_new_source_passthrough)
{
    // Source-side passthrough moves a route start to the next resource column.
    if (!from || !from->tile.peer || !from->cell_ref.peer) {
        return false;
    }
    if (forcesFabricOutput(from)) {
        return false;
    }
    Tile& tile = *from->tile;
    ElementType type = instElementType(*from);
    if (!isSourcePassthroughPort(type, from_port)) {
        return false;
    }
    int bit = elementBitFromPlacedPos(type, from->pos);
    if (bit < 0 || !hasNeighbor(tile, type, bit, true)) {
        return false;
    }

    rtl::Conn* source_out = findConn(*from, from_port, rtl::Port::PORT_OUT);
    if (!source_out) {
        source_out = firstOutputConn(*from);
    }
    if (!source_out || !source_out->port_ref.peer) {
        return false;
    }

    auto use_existing_passthrough = [&](rtl::Inst* pass) {
        if (!pass || generatedPassthroughKind(pass) != "source") {
            return false;
        }
        if (!pass->tile.peer) {
            rtl::Conn* pass_in = firstInputConn(*pass);
            std::string pass_input = pass_in && pass_in->port_ref.peer
                ? pass_in->port_ref->makeName() : std::string{};
            ElementType pass_type = instElementType(*pass);
            std::optional<NeighborElement> neighbor = firstFreeNeighbor(
                tile, type, bit, true, pass_input, pass_type);
            if (!neighbor || !placeGeneratedAtElement(tile, *pass, neighbor->type, neighbor->bit)) {
                return false;
            }
            refreshPassthroughVoidNets(tile);
        }
        else if (pass->tile.peer != from->tile.peer) {
            return false;
        }
        rtl::Conn* pass_out = firstOutputConn(*pass);
        if (!pass_out || !pass_out->port_ref.peer) {
            return false;
        }
        from = pass;
        from_port = pass_out->port_ref->makeName();
        if (rtl::Module* module = ownerModule(*from)) {
            net = findNetInModuleByDesignator(*module, pass_out->port_ref->designator);
        }
        return true;
    };

    // Generated chains are stable after insertion, so reuse the exact next hop in O(1).
    if (source_out->route_endpoint && use_existing_passthrough(source_out->route_endpoint)) {
        return true;
    }

    std::vector<RefBase<Referable<rtl::Conn>>*> old_sinks = rtl::Conn::getSinks(*source_out);
    for (auto* sink_ref : old_sinks) {
        rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
        if (!sink_conn || !sink_conn->inst_ref.peer || !sink_conn->inst_ref->cell_ref.peer) {
            continue;
        }
        if (generatedPassthroughKind(sink_conn->inst_ref.peer) == "source") {
            rtl::Inst* pass = sink_conn->inst_ref.peer;
            if (use_existing_passthrough(pass)) {
                source_out->route_endpoint = pass;
                return true;
            }
        }
    }

    if (!allow_new_source_passthrough) {
        return false;
    }
    std::optional<NeighborElement> neighbor = firstFreeNeighbor(tile, type, bit, true);
    if (!neighbor) {
        return false;
    }
    rtl::Module* module = ownerModule(*from);
    if (!module) {
        return false;
    }
    int route_designator = source_out->port_ref->designator;
    if (route_designator < 0) {
        return false;
    }
    int void_designator = nextGeneratedDesignator(*module);
    rtl::Net* route_net = findNetInModuleByDesignator(*module, route_designator);
    if (!route_net) {
        return false;
    }

    rtl::Inst* pass = makeGeneratedPassthroughInst(
        *from, neighbor->type, neighbor->input_port);
    pass->cell_ref->attributes["scalepnr_passthrough"] = "source";
    rtl::Conn* pass_in = firstInputConn(*pass);
    rtl::Conn* pass_out = firstOutputConn(*pass);
    PNR_ASSERT(pass_in && pass_out, "generated source passthrough is missing ports");

    connectConns(*pass_in, *source_out, void_designator);
    pass_out->port_ref->designator = route_designator;
    // Transfer the complete logical fanout in one linear operation.
    rtl::Conn::fromBase(*source_out).movePeersTo(
        rtl::Conn::fromBase(*pass_out), &rtl::Conn::fromBase(*pass_in));
    for (auto* sink_ref : old_sinks) {
        rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
        if (sink_conn && sink_conn != pass_in) {
            invalidateInputJointReservations(*sink_conn);
        }
    }
    if (!placeGeneratedAtElement(tile, *pass, neighbor->type, neighbor->bit)) {
        source_out->port_ref->designator = route_designator;
        // Restore the logical fanout atomically when physical placement rejects the endpoint.
        rtl::Conn::fromBase(*pass_out).movePeersTo(rtl::Conn::fromBase(*source_out));
        for (auto* sink_ref : old_sinks) {
            rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            if (sink_conn && sink_conn != pass_in) {
                invalidateInputJointReservations(*sink_conn);
            }
        }
        if (pass->parent_ref.peer && !pass->parent_ref->insts.empty()
            && &pass->parent_ref->insts.back() == pass) {
            pass->parent_ref->insts.pop_back();
        }
        return false;
    }
    source_out->route_endpoint = pass;

    // This function changes topology only. The routing scheduler atomically
    // unroutes and retargets the complete source tree after this returns.

    appendGeneratedNet(*module,
        std::format("{}.$scalepnr_passthrough_in{}", from->makeName(), void_designator),
        void_designator, true);
    refreshPassthroughVoidNets(tile);
    from = pass;
    from_port = pass_out->port_ref->makeName();
    net = route_net;
    return true;
}

bool ensureTargetPassthrough(rtl::Inst*& to, std::string& to_port, rtl::Net*& net)
{
    // Target-side passthrough moves a route end to the previous resource column.
    if (!to || !to->tile.peer || !to->cell_ref.peer) {
        return false;
    }
    Tile& tile = *to->tile;
    ElementType type = instElementType(*to);
    auto log_distributed_failure = [&](const char* reason, int bit) {
        if (!net || !net->distributed_source ||
            std::getenv("SCALEPNR_VCC_DEBUG") == nullptr) {
            return;
        }
        std::string predecessors;
        if (bit >= 0 && bit < ELEMENT_BITMAP_BITS) {
            for (int type_index = 0; type_index < ELEMENT_TYPE_COUNT; ++type_index) {
                ElementType predecessor = static_cast<ElementType>(type_index);
                uint16_t linked = linkedNeighborMask(tile, type, bit, predecessor, false);
                if (linked == 0) {
                    continue;
                }
                if (!predecessors.empty()) {
                    predecessors += "; ";
                }
                predecessors += std::format("{} linked=0x{:04x} free=0x{:04x}",
                    elementTypeName(predecessor), linked,
                    tile.elements_free[predecessor]);
                uint16_t occupied = linked & static_cast<uint16_t>(~tile.elements_free[predecessor]);
                while (occupied) {
                    int predecessor_bit = std::countr_zero(static_cast<unsigned>(occupied));
                    occupied &= static_cast<uint16_t>(occupied - 1);
                    rtl::Inst* owner = elementInstAt(tile, predecessor, predecessor_bit);
                    predecessors += std::format(" bit{}={}", predecessor_bit,
                        owner ? owner->makeName() : std::string("<unowned>"));
                }
            }
        }
        PNR_LOG1("FPGA",
            "distributed target passthrough unavailable: sink='{}' type='{}' "
            "port='{}' tile=({},{}) pos={} element={} bit={} reason={} predecessors=[{}]",
            to->makeName(), to->cell_ref->type, to_port, tile.coord.x,
            tile.coord.y, to->pos, elementTypeName(type), bit, reason,
            predecessors);
    };
    if (!isTargetPassthroughPort(type, to_port)) {
        log_distributed_failure("port is fabric-facing", -1);
        return false;
    }
    int bit = elementBitFromPlacedPos(type, to->pos);
    if (bit < 0 || !hasNeighbor(tile, type, bit, false)) {
        log_distributed_failure("no loaded predecessor link", bit);
        return false;
    }

    rtl::Conn* target_in = findConn(*to, to_port, rtl::Port::PORT_IN);
    if (!target_in) {
        target_in = firstInputConn(*to);
    }
    if (!target_in || !target_in->port_ref.peer) {
        log_distributed_failure("missing target input connection", bit);
        return false;
    }
    rtl::Conn* driver = target_in->follow();
    if (!driver || !driver->port_ref.peer) {
        log_distributed_failure("missing logical driver", bit);
        return false;
    }
    if (generatedPassthroughKind(driver->inst_ref.peer) == "target") {
        rtl::Inst* pass = driver->inst_ref.peer;
        if (!pass->tile.peer) {
            ElementType pass_type = instElementType(*pass);
            std::optional<NeighborElement> neighbor = firstFreeNeighbor(tile, type, bit, false,
                target_in->port_ref->makeName(), pass_type);
            if (!neighbor || !placeGeneratedAtElement(tile, *pass, neighbor->type, neighbor->bit)) {
                return false;
            }
            refreshPassthroughVoidNets(tile);
        }
        else if (pass->tile.peer != to->tile.peer) {
            return false;
        }
        rtl::Conn* pass_in = firstInputConn(*pass);
        if (pass_in && pass_in->port_ref.peer) {
            to = pass;
            to_port = pass_in->port_ref->makeName();
            if (rtl::Module* module = ownerModule(*to)) {
                net = findNetInModuleByDesignator(*module, pass_in->port_ref->designator);
            }
            return true;
        }
    }

    std::optional<NeighborElement> neighbor = firstFreeNeighbor(tile, type, bit, false,
        target_in->port_ref->makeName());
    if (!neighbor && net && net->distributed_source) {
        // Keep the required physical predecessor as an unplaced endpoint. The
        // Moving transaction will rehome it with the real packed sink cluster.
        neighbor = firstFreeNeighbor(tile, type, bit, false,
            target_in->port_ref->makeName(), std::nullopt, false);
    }
    if (!neighbor) {
        log_distributed_failure("all compatible predecessor lanes are occupied", bit);
        return false;
    }
    rtl::Module* module = ownerModule(*to);
    if (!module) {
        log_distributed_failure("missing owner module", bit);
        return false;
    }
    int route_designator = target_in->port_ref->designator;
    // Protected infrastructure routes may originate outside the module's
    // ordinary net table, so their caller-supplied route object is canonical.
    rtl::Net* route_net = net && net->route_protected
        ? net : findNetInModuleByDesignator(*module, route_designator);
    if (route_net == net && !route_net->designators.empty()) {
        route_designator = route_net->designators.front();
    }
    if (route_designator < 0 || !route_net) {
        log_distributed_failure("missing route-net designator", bit);
        return false;
    }
    int void_designator = nextGeneratedDesignator(*module);

    rtl::Inst* pass = makeGeneratedPassthroughInst(*to, neighbor->type);
    pass->cell_ref->attributes["scalepnr_passthrough"] = "target";
    rtl::Conn* pass_in = firstInputConn(*pass);
    rtl::Conn* pass_out = firstOutputConn(*pass);
    PNR_ASSERT(pass_in && pass_out, "generated target passthrough is missing ports");

    connectConns(*pass_in, *driver, route_designator);
    connectConns(*target_in, *pass_out, void_designator);
    if (!placeGeneratedAtElement(tile, *pass, neighbor->type, neighbor->bit)) {
        log_distributed_failure("selected predecessor rejected placement", bit);
        if (route_net->distributed_source) {
            // Preserve the physical endpoint identity even when its current
            // packed tile cannot host it. Generic Moving will relocate the
            // real sink cluster and rehome this unplaced endpoint there.
            appendGeneratedNet(*module,
                std::format("{}.$scalepnr_passthrough_out{}", pass->makeName(), void_designator),
                void_designator, true);
            refreshPassthroughVoidNets(tile);
            to = pass;
            to_port = pass_in->port_ref->makeName();
            net = route_net;
            return true;
        }
        target_in->port_ref->designator = route_designator;
        target_in->set(&rtl::Conn::fromBase(*driver));
        invalidateInputJointReservations(*target_in);
        if (pass->parent_ref.peer && !pass->parent_ref->insts.empty()
            && &pass->parent_ref->insts.back() == pass) {
            pass->parent_ref->insts.pop_back();
        }
        return false;
    }
    appendGeneratedNet(*module,
        std::format("{}.$scalepnr_passthrough_out{}", pass->makeName(), void_designator),
        void_designator, true);
    refreshPassthroughVoidNets(tile);
    to = pass;
    to_port = pass_in->port_ref->makeName();
    net = route_net;
    return true;
}

rtl::Inst* carryInSite(Tile& tile, int site)
{
    // Locate the carry primitive currently assigned to the selected site.
    for (rtl::Inst* inst : assignedInsts(tile)) {
        if (inst && isCarry(*inst) && siteIndexFromPlacedPos(inst->pos) == site) {
            return inst;
        }
    }
    return nullptr;
}

bool carryLutSlotCompatible(Tile& tile, rtl::Inst* inst, int pos)
{
    // Keep LUT and carry placement in the same slot when their pins are connected.
    int site = siteIndexFromPlacedPos(pos);
    int bel = belIndexFromPlacedPos(pos);

    if (isLut(*inst)) {
        rtl::Inst* carry = carryInSite(tile, site);
        if (!carry) {
            return true;
        }
        rtl::Conn* s_driver = carryInputDriver(*carry, "S", bel);
        return !s_driver || hasOutputConn(*inst, s_driver);
    }

    if (isCarry(*inst)) {
        for (int bel = 0; bel < 4; ++bel) {
            rtl::Conn* s_driver = carryInputDriver(*inst, "S", bel);
            if (!s_driver) {
                continue;
            }
            rtl::Inst* lut = lutAtBel(tile, site, bel);
            if (lut && !hasOutputConn(*lut, s_driver)) {
                return false;
            }
        }
    }

    return true;
}

bool canHost(Tile& tile, rtl::Inst* inst, int pos)
{
    // Reject placements that the abstract tile type cannot host.
    if (!tile.tile_type) {
        return false;
    }

    if (maybeInstElementType(*inst)) {
        return tileTypeHasLogicElements(*tile.tile_type) && carryLutSlotCompatible(tile, inst, pos);
    }
    return true;
}

bool useResourcePinNameFallback(const std::string& type)
{
    // Restrict resource-pin-name fallback to logic primitives with packed site pins.
    return type == "INV"
        || type.find("LUT") == 0
        || type.find("FD") == 0
        || type.find("CARRY") == 0
        || type.find("MUX") == 0
        || type == "IBUF"
        || type == "OBUF";
}

std::string normalizedResourcePinName(std::string type, std::string port, int pos)
{
    // Convert generic cell ports and placement position to a tile resource pin name.
    if (type == "INV") {
        type = "LUT6";
        if (port == "I") {
            port = "I0";
        }
    }
    int bit = extractIndexedPort(port);
    if (type.find("LUT") == 0 && bit >= 0 && port == "I") {
        port = "I" + std::to_string(bit);
    }
    if (type.find("CARRY") == 0 && bit >= 0) {
        if (port == "DI" || port == "S" || port == "O") {
            port += std::to_string(bit);
        }
        else if (port == "CO") {
            port = "C" + std::to_string(bit);
        }
    }
    if (type.find("MUX") == 0 && bit >= 0 && port == "I") {
        port = "I" + std::to_string(bit);
    }
    if (type == "IBUF" && port == "O") {
        return "I";
    }
    if (type == "OBUF" && port == "I") {
        return "O";
    }

    static constexpr char bel_prefix[4] = {'A', 'B', 'C', 'D'};
    int bel = belIndexFromPlacedPos(pos);
    if (type.find("MUX") == 0 && (port == "I0" || port == "I1" || port == "O")) {
        bel = port == "O"
            ? muxOutputBelFromPlacedPos(type, pos)
            : muxDataBelFromPlacedPos(type, port, pos);
    }
    if (type.find("MUX") == 0 && port == "S") {
        bel = muxControlBelFromPlacedPos(type, pos);
    }
    char prefix = bel >= 0 && bel < 4 ? bel_prefix[bel] : 'A';

    if (type.find("LUT") == 0) {
        if (port == "I0" || port == "A1") return std::string{prefix} + "1";
        if (port == "I1" || port == "A2") return std::string{prefix} + "2";
        if (port == "I2" || port == "A3") return std::string{prefix} + "3";
        if (port == "I3" || port == "A4") return std::string{prefix} + "4";
        if (port == "I4" || port == "A5") return std::string{prefix} + "5";
        if (port == "I5" || port == "A6") return std::string{prefix} + "6";
        if (type == "LUT1" && port == "O") return std::string{prefix} + "MUX";
        if (port == "O6" || port == "O") return std::string{prefix};
        if (port == "O5") return std::string{prefix} + "X";
    }
    if (type.find("FD") == 0) {
        if (port == "D") return std::string{prefix} + "X";
        if (port == "Q") return std::string{prefix} + "Q";
        if (port == "R" || port == "S" || port == "CLR" || port == "PRE" || port == "SRST" || port == "ARST") return "SR";
        if (port == "C") return "CLK";
        if (port == "CE" || port == "EN") return "CE";
    }
    if (type.find("MUX") == 0) {
        if (port == "O" || port == "I0" || port == "I1") return std::string{prefix} + "MUX";
        if (port == "S") return std::string{prefix} + "X";
    }
    return port;
}

std::string modeledResourcePinName(const TileType* tile_type, std::string type, std::string port, int pos)
{
    // Prefer the loaded site model when the placement position selects a concrete site.
    std::string normalized = normalizedResourcePinName(type, port, pos);
    if (technology::mappedSitePinName) {
        normalized = technology::mappedSitePinName(type, port, pos, normalized);
    }
    const SiteModel* site = tile_type ? tile_type->siteForPlacedPos(pos) : nullptr;
    if (!site) {
        return normalized;
    }
    // Keep routing permissive when a database site lacks a modeled pin.
    return normalized;
}

int modeledSitePos(const TileType* tile_type, int pos)
{
    // Convert placement position into the site coordinate used by tile-pin maps.
    return tile_type ? tile_type->sitePosForPlacedPos(pos) : -1;
}

int modeledResourceNodeNum(const TileType* tile_type, const std::string& type, int pos, int base_node)
{
    // Tile pin maps store resource nodes with the modeled site position encoded.
    if (base_node < 0 || !tile_type || !useResourcePinNameFallback(type)) {
        return base_node;
    }
    int site_pos = modeledSitePos(tile_type, pos);
    return site_pos >= 0 ? base_node + site_pos*256 : base_node;
}

bool endpointDebugEnabled()
{
    // Optional endpoint trace for diagnosing database-to-local pin resolution.
    return std::getenv("SCALEPNR_ENDPOINT_DEBUG") != nullptr;
}

}

NodeMask fpga::packedInputJointReservations(Tile& route_tile, rtl::Inst* except_inst,
                                            const std::string& except_port)
{
    // Reserve only joints present on every physical entry path of another packed input.
    if (!route_tile.cb_type) {
        return {};
    }
    NodeMask reserved{};
    rtl::Conn* except_driver = nullptr;
    if (except_inst) {
        for (rtl::Conn& conn : except_inst->conns) {
            if (conn.port_ref.peer && conn.port_ref->type == rtl::Port::PORT_IN
                && conn.port_ref->makeName() == except_port) {
                except_driver = conn.follow();
                break;
            }
        }
    }

    ensureInputJointReservations(route_tile);
    // Report the exact packed drivers behind a focused terminal-joint conflict.
    const char* debug_reservations = std::getenv("SCALEPNR_DEBUG_INPUT_JOINTS");
    bool trace_reservations = debug_reservations && except_inst
        && (std::string(debug_reservations) == "*"
            || except_inst->makeName(250).find(debug_reservations) != std::string::npos);
    for (const auto& [driver, joints] : route_tile.input_joint_reservations) {
        if (trace_reservations) {
            PNR_LOG1("FPGA",
                "packed input joint reservation: target='{}'/'{}', route_tile=({},{}), "
                "driver='{}', joints={}, excluded={}",
                except_inst->makeName(250), except_port,
                route_tile.coord.x, route_tile.coord.y,
                driver ? driver->makeName(nullptr, 250) : std::string{},
                joints.str(), driver == except_driver);
        }
        if (!except_driver || driver != except_driver) {
            reserved |= joints;
        }
    }
    return reserved;
}

bool fpga::preparePassthroughRouteEndpoints(rtl::Inst*& from, std::string& from_port,
                                            rtl::Inst*& to, std::string& to_port,
                                            rtl::Net*& net, bool allow_new_source_passthrough)
{
    bool changed = false;
    bool trace = std::getenv("SCALEPNR_PASSTHROUGH_TRACE") != nullptr;
    if (trace) {
        PNR_LOG1("FPGA", "passthrough prepare start: from='{}'/'{}', to='{}'/'{}', allow_source={}",
            from ? from->makeName() : std::string{}, from_port,
            to ? to->makeName() : std::string{}, to_port,
            allow_new_source_passthrough);
    }
    if (trace) {
        PNR_LOG1("FPGA", "passthrough source start: from='{}'/'{}'",
            from ? from->makeName() : std::string{}, from_port);
    }
    if (ensureSourcePassthrough(from, from_port, net, allow_new_source_passthrough)) {
        changed = true;
    }
    if (trace) {
        PNR_LOG1("FPGA", "passthrough source done: changed={}, from='{}'/'{}'",
            changed, from ? from->makeName() : std::string{}, from_port);
        PNR_LOG1("FPGA", "passthrough target start: to='{}'/'{}'",
            to ? to->makeName() : std::string{}, to_port);
    }
    if (ensureTargetPassthrough(to, to_port, net)) {
        changed = true;
    }
    if (trace) {
        PNR_LOG1("FPGA", "passthrough prepare done: changed={}, from='{}'/'{}', to='{}'/'{}'",
            changed,
            from ? from->makeName() : std::string{}, from_port,
            to ? to->makeName() : std::string{}, to_port);
    }
  return changed;
}

bool fpga::rehomeGeneratedPassthrough(rtl::Inst& inst, std::string* fail_reason)
{
    // Reuse the generated endpoint identity while moving its tile-local packed lane.
    auto fail = [&](const std::string& reason) {
        if (fail_reason) {
            *fail_reason = reason;
        }
        return false;
    };
    std::string_view kind = generatedPassthroughKind(&inst);
    if (inst.tile.peer || kind.empty()) {
        return inst.tile.peer ? true : fail("instance is not a generated endpoint");
    }
    ElementType pass_type = instElementType(inst);
    if (kind == "source") {
        rtl::Conn* input = firstInputConn(inst);
        rtl::Conn* driver = input ? input->follow() : nullptr;
        rtl::Inst* anchor = driver ? driver->inst_ref.peer : nullptr;
        if (!anchor || !anchor->tile.peer || !anchor->cell_ref.peer) {
            return fail("source endpoint has no placed input driver");
        }
        ElementType anchor_type = instElementType(*anchor);
        int anchor_bit = elementBitFromPlacedPos(anchor_type, anchor->pos);
        if (anchor_bit < 0 || anchor_bit >= ELEMENT_BITMAP_BITS) {
            return fail(std::format("source endpoint anchor has invalid element bit; anchor='{}' type={} pos={}",
                anchor->makeName(), elementTypeName(anchor_type), anchor->pos));
        }
        std::string input_port = input && input->port_ref.peer
            ? input->port_ref->makeName() : std::string{};
        std::optional<NeighborElement> neighbor = firstFreeNeighbor(
            *anchor->tile, anchor_type, anchor_bit, true, input_port, pass_type);
        if (!neighbor) {
            // Report the loaded link and occupancy masks that rejected source rehome.
            uint16_t linked = 0;
            for (int neighbor_bit = 0; neighbor_bit < ELEMENT_BITMAP_BITS; ++neighbor_bit) {
                if ((anchor->tile->elements_right[anchor_type][neighbor_bit]
                        & bit16(anchor_bit)) != 0) {
                    linked |= bit16(neighbor_bit);
                }
            }
            return fail(std::format(
                "source endpoint has no free linked element lane; anchor='{}' type={} tile=({},{}) pos={} bit={} endpoint_type={} linked=0x{:04x} free=0x{:04x}",
                anchor->makeName(), elementTypeName(anchor_type),
                anchor->tile->coord.x, anchor->tile->coord.y, anchor->pos, anchor_bit,
                elementTypeName(pass_type), linked, anchor->tile->elements_free[pass_type]));
        }
        if (!placeGeneratedAtElement(*anchor->tile, inst, neighbor->type, neighbor->bit)) {
            return fail("source endpoint linked lane rejected placement");
        }
        return true;
    }
    if (kind == "target") {
        rtl::Conn* output = firstOutputConn(inst);
        if (!output) {
            return fail("target endpoint has no output connection");
        }
        std::string attempts;
        for (auto* sink_ref : rtl::Conn::getSinks(*output)) {
            rtl::Conn* sink = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            rtl::Inst* anchor = sink ? sink->inst_ref.peer : nullptr;
            if (!anchor || !anchor->tile.peer || !anchor->cell_ref.peer) {
                continue;
            }
            ElementType anchor_type = instElementType(*anchor);
            int anchor_bit = elementBitFromPlacedPos(anchor_type, anchor->pos);
            if (anchor_bit < 0 || anchor_bit >= ELEMENT_BITMAP_BITS) {
                continue;
            }
            std::string sink_port = sink->port_ref.peer
                ? sink->port_ref->makeName() : std::string{};
            std::optional<NeighborElement> neighbor = firstFreeNeighbor(
                *anchor->tile, anchor_type, anchor_bit, false,
                sink_port, pass_type);
            if (neighbor && placeGeneratedAtElement(
                    *anchor->tile, inst, neighbor->type, neighbor->bit)) {
                return true;
            }
            uint16_t linked = linkedNeighborMask(
                *anchor->tile, anchor_type, anchor_bit, pass_type, false);
            uint16_t compatible = 0;
            uint16_t route_blocked = 0;
            for (int pass_bit = 0; pass_bit < ELEMENT_BITMAP_BITS; ++pass_bit) {
                if ((linked & bit16(pass_bit)) == 0
                    || !strictLocalChainLaneMatches(
                        pass_type, pass_bit, anchor_type, anchor_bit, sink_port)) {
                    continue;
                }
                compatible |= bit16(pass_bit);
                int pos = placedPosFromElementBit(pass_type, pass_bit);
                auto [input_port, output_port] = passthroughPorts(pass_type);
                NodeMask route_nodes = anchor->tile->getPinNodes(
                    passthroughCellType(pass_type), input_port, pos);
                if (route_nodes != NodeMask{}
                    && (route_nodes & anchor->tile->pin_state.leased_nodes) != NodeMask{}) {
                    route_blocked |= bit16(pass_bit);
                }
            }
            if (!attempts.empty()) {
                attempts += "; ";
            }
            attempts += std::format(
                "anchor='{}' tile=({},{}) type={} pos={} bit={} port={} linked=0x{:04x} compatible=0x{:04x} free=0x{:04x} route_blocked=0x{:04x}",
                anchor->makeName(), anchor->tile->coord.x, anchor->tile->coord.y,
                elementTypeName(anchor_type), anchor->pos, anchor_bit, sink_port,
                linked, compatible, anchor->tile->elements_free[pass_type], route_blocked);
        }
        return fail(std::format(
            "target endpoint has no free linked lane beside a placed sink; endpoint_type={} attempts=[{}]",
            elementTypeName(pass_type), attempts));
    }
    return fail(std::format("generated endpoint has unknown kind '{}'", kind));
}

bool fpga::inputLocalReservedByDriver(Tile& route_tile, int local,
                                      rtl::Conn* driver)
{
    // Keep the public query numeric while the reservation cache stays internal.
    return inputLocalReservedByDriverImpl(route_tile, local, driver);
}

const char* fpga::elementTypeName(ElementType type)
{
    switch (type) {
    case ELEMENT_LUT5: return "LUT5";
    case ELEMENT_LUT1: return "LUT1";
    case ELEMENT_MUXF7: return "MUXF7";
    case ELEMENT_MUXF8: return "MUXF8";
    case ELEMENT_CARRY: return "CARRY";
    case ELEMENT_FD: return "FD";
    default: return "UNKNOWN";
    }
}

void TileType::rebuildElementsFromSites()
{
    // Derive abstract element columns from loaded site pins instead of tile names.
    elements.clear();
    int site_index = 0;
    for (const SiteModel& site : sites) {
        if (!siteHasLogicOutputLanes(site)) {
            ++site_index;
            continue;
        }
        uint16_t bel_mask = detectedSiteBelMask(site);
        if (bel_mask == 0) {
            ++site_index;
            continue;
        }
        for (int bel = 0; bel < 4; ++bel) {
            if ((bel_mask & bit16(bel)) == 0) {
                continue;
            }
            uint16_t bit = static_cast<uint16_t>(site_index*4 + bel);
            if (bit >= ELEMENT_BITMAP_BITS) {
                continue;
            }
            addElement(*this, std::format("{}_LUT5{}", site.name, bel), ELEMENT_LUT5, bit, elementColumn(ELEMENT_LUT5));
            addElement(*this, std::format("{}_LUT1{}", site.name, bel), ELEMENT_LUT1, bit, elementColumn(ELEMENT_LUT1));
            uint16_t fd_bit = static_cast<uint16_t>(site_index*8 + bel);
            uint16_t fd2_bit = static_cast<uint16_t>(site_index*8 + 4 + bel);
            if (fd_bit < ELEMENT_BITMAP_BITS) {
                addElement(*this, std::format("{}_FD{}", site.name, bel), ELEMENT_FD, fd_bit, elementColumn(ELEMENT_FD));
            }
            if (fd2_bit < ELEMENT_BITMAP_BITS) {
                addElement(*this, std::format("{}_FD2{}", site.name, bel), ELEMENT_FD, fd2_bit, elementColumn(ELEMENT_FD));
            }
        }
        uint16_t a = static_cast<uint16_t>(site_index*4);
        uint16_t c = static_cast<uint16_t>(site_index*4 + 2);
        if (a < ELEMENT_BITMAP_BITS && ((bel_mask & 0x03) == 0x03 || siteHasPort(site, "AMUX"))) {
            addElement(*this, site.name + "_MUXF7_0", ELEMENT_MUXF7, a, elementColumn(ELEMENT_MUXF7));
            connectElements(*this, ELEMENT_LUT5, a, ELEMENT_MUXF7, a);
            connectElements(*this, ELEMENT_LUT5, static_cast<uint16_t>(a + 1), ELEMENT_MUXF7, a);
            connectElements(*this, ELEMENT_LUT1, a, ELEMENT_MUXF7, a);
            connectElements(*this, ELEMENT_LUT1, static_cast<uint16_t>(a + 1), ELEMENT_MUXF7, a);
        }
        if (c < ELEMENT_BITMAP_BITS && ((bel_mask & 0x0c) == 0x0c || siteHasPort(site, "CMUX"))) {
            addElement(*this, site.name + "_MUXF7_1", ELEMENT_MUXF7, c, elementColumn(ELEMENT_MUXF7));
            connectElements(*this, ELEMENT_LUT5, c, ELEMENT_MUXF7, c);
            connectElements(*this, ELEMENT_LUT5, static_cast<uint16_t>(c + 1), ELEMENT_MUXF7, c);
            connectElements(*this, ELEMENT_LUT1, c, ELEMENT_MUXF7, c);
            connectElements(*this, ELEMENT_LUT1, static_cast<uint16_t>(c + 1), ELEMENT_MUXF7, c);
        }
        if (a < ELEMENT_BITMAP_BITS && (siteHasPort(site, "AMUX") || siteHasPort(site, "BMUX") || siteHasPort(site, "CMUX") || siteHasPort(site, "DMUX"))) {
            uint16_t fd_a = static_cast<uint16_t>(site_index*8);
            uint16_t fd2_a = static_cast<uint16_t>(site_index*8 + 4);
            addElement(*this, site.name + "_MUXF8", ELEMENT_MUXF8, a, elementColumn(ELEMENT_MUXF8));
            connectElements(*this, ELEMENT_MUXF7, a, ELEMENT_MUXF8, a);
            connectElements(*this, ELEMENT_MUXF7, c, ELEMENT_MUXF8, a);
            if (fd_a < ELEMENT_BITMAP_BITS) {
                connectElements(*this, ELEMENT_MUXF8, a, ELEMENT_FD, fd_a);
            }
            if (fd2_a < ELEMENT_BITMAP_BITS) {
                connectElements(*this, ELEMENT_MUXF8, a, ELEMENT_FD, fd2_a);
            }
        }
        if (a < ELEMENT_BITMAP_BITS && (siteHasPort(site, "CIN") || siteHasPort(site, "COUT"))) {
            addElement(*this, site.name + "_CARRY", ELEMENT_CARRY, a, elementColumn(ELEMENT_CARRY));
        }
        ++site_index;
    }
    std::sort(elements.begin(), elements.end(), [](const Element& a, const Element& b) {
        if (a.elements_to_left != b.elements_to_left) return a.elements_to_left < b.elements_to_left;
        if (a.type != b.type) return a.type < b.type;
        return a.bitmap_pos < b.bitmap_pos;
    });
    if (packDebugEnabled() && !elements.empty()) {
        std::array<uint16_t, ELEMENT_TYPE_COUNT> masks{};
        for (const Element& element : elements) {
            masks[element.type] |= bit16(element.bitmap_pos);
        }
        std::fprintf(stderr, "pack-debug loaded TileType=%s sites=%zu elements=%zu", name.c_str(), sites.size(), elements.size());
        for (int type_index = 0; type_index < ELEMENT_TYPE_COUNT; ++type_index) {
            ElementType type = static_cast<ElementType>(type_index);
            std::fprintf(stderr, " %s=0x%04x", elementTypeName(type), masks[type_index]);
        }
        std::fprintf(stderr, "\n");
        printElementLinks(*this);
    }
}

NodeMask Tile::getPinNodes(const std::string& type, const std::string& port, int pos) const
{
    int local = const_cast<Tile*>(this)->getNodeNum(type, port, pos);
    if (tile_type) {
        if (useResourcePinNameFallback(type)) {
            // Packed logic pins are best resolved by site pin identity.
            NodeMask nodes = tile_type->pin_map.getNodesForPin(TILE_PIN_INPUT, modeledResourcePinName(tile_type, type, port, pos),
                                                            modeledSitePos(tile_type, pos),
                                                            cb_type ? cb_type->name : std::string{});
            if (endpointDebugEnabled()) {
                PNR_LOG1("FPGA", "endpoint input tile='{}' cb='{}' type='{}' port='{}' pos={} site_pos={} pin='{}' nodes={}",
                    makeName(), cb_type ? cb_type->name : std::string{}, type, port, pos,
                    modeledSitePos(tile_type, pos), modeledResourcePinName(tile_type, type, port, pos), nodes.str());
            }
            if (nodes != NodeMask{}) {
                return nodes;
            }
        }
        int resource_node = modeledResourceNodeNum(tile_type, type, pos, local);
        NodeMask nodes = resource_node < 0 ? NodeMask{} : tile_type->pin_map.getInputNodes(resource_node);
        if (nodes != NodeMask{}) {
            return nodes;
        }
        nodes = tile_type->pin_map.getNodes(type, port, pos);
        if (nodes != NodeMask{}) {
            return nodes;
        }
    }

    if (tile_type && useResourcePinNameFallback(type)) {
        if (local >= 0 && cb_type && (cb_type->local_input_nodes & (NodeMask{0,1} << local)) != NodeMask{}) {
            return NodeMask{0,1} << local;
        }
        // Preserve abstract fallback only for tile types without loaded site endpoint models.
        if (local >= 0 && tile_type->pin_map.endpoint_route_refs.empty()) {
            return NodeMask{0,1} << local;
        }
        return NodeMask{};
    }
    return local < 0 ? NodeMask{} : (NodeMask{0,1} << local);
}

NodeMask Tile::getOutputPinNodes(const std::string& type, const std::string& port, int pos) const
{
    int local = const_cast<Tile*>(this)->getNodeNum(type, port, pos);
    if (tile_type) {
        if (useResourcePinNameFallback(type)) {
            // Packed logic outputs are best resolved by site pin identity.
            NodeMask nodes = tile_type->pin_map.getNodesForPin(TILE_PIN_OUTPUT, modeledResourcePinName(tile_type, type, port, pos),
                                                            modeledSitePos(tile_type, pos),
                                                            cb_type ? cb_type->name : std::string{});
            if (endpointDebugEnabled()) {
                PNR_LOG1("FPGA", "endpoint output tile='{}' cb='{}' type='{}' port='{}' pos={} site_pos={} pin='{}' nodes={}",
                    makeName(), cb_type ? cb_type->name : std::string{}, type, port, pos,
                    modeledSitePos(tile_type, pos), modeledResourcePinName(tile_type, type, port, pos), nodes.str());
            }
            if (nodes != NodeMask{}) {
                return nodes;
            }
        }
        int resource_node = modeledResourceNodeNum(tile_type, type, pos, local);
        NodeMask nodes = resource_node < 0 ? NodeMask{} : tile_type->pin_map.getOutputNodes(resource_node);
        if (nodes != NodeMask{}) {
            return nodes;
        }
    }

    return local < 0 ? NodeMask{} : (NodeMask{0,1} << local);
}

NodeMask Tile::getPinNodesForRouteType(const std::string& type, const std::string& port, int pos,
                                   TilePinNameType dir, const std::string& route_type) const
{
    return getPinNodesForRouteType(type, port, pos, dir, route_type, Coord{0, 0});
}

NodeMask Tile::getPinNodesForRouteType(const std::string& type, const std::string& port, int pos,
                                   TilePinNameType dir, const std::string& route_type, Coord route_delta) const
{
    // Endpoint route filtering lets adjacent route tiles own only their exact site-local nodes.
    if (!tile_type || route_type.empty()) {
        return dir == TILE_PIN_OUTPUT ? getOutputPinNodes(type, port, pos) : getPinNodes(type, port, pos);
    }

    if (useResourcePinNameFallback(type)) {
        // Route-typed endpoint refs are exact ownership data; do not fall back to unrelated route tiles.
        bool strict_route_type = !tile_type->pin_map.endpoint_route_refs.empty();
        NodeMask nodes = tile_type->pin_map.getNodesForPin(dir, modeledResourcePinName(tile_type, type, port, pos),
                                                       modeledSitePos(tile_type, pos), route_type, strict_route_type,
                                                       &route_delta);
        if (endpointDebugEnabled()) {
            PNR_LOG1("FPGA", "endpoint route_type tile='{}' route='{}' delta=({}, {}) dir={} type='{}' port='{}' pos={} nodes={}",
                makeName(), route_type, route_delta.x, route_delta.y, static_cast<int>(dir), type, port, pos, nodes.str());
        }
        return nodes;
    }

    int local = const_cast<Tile*>(this)->getNodeNum(type, port, pos);
    int resource_node = modeledResourceNodeNum(tile_type, type, pos, local);
    if (resource_node >= 0) {
        NodeMask nodes = dir == TILE_PIN_OUTPUT
            ? tile_type->pin_map.getOutputNodes(resource_node)
            : tile_type->pin_map.getInputNodes(resource_node);
        if (nodes != NodeMask{}) {
            return nodes;
        }
    }
    return NodeMask{};
}

int Tile::getResourceNodeNum(const std::string& type, const std::string& port, int pos, TilePinNameType dir, int local) const
{
    // Resolve endpoint identity after routing selects the concrete local node.
    int preferred = modeledResourceNodeNum(tile_type, type, pos,
        const_cast<Tile*>(this)->getNodeNum(type, port, pos));
    if (!tile_type) {
        return preferred;
    }
    return tile_type->pin_map.findResourceNode(dir,
        useResourcePinNameFallback(type) ? modeledResourcePinName(tile_type, type, port, pos) : std::string{},
        local, preferred, modeledSitePos(tile_type, pos));
}

bool Tile::leasePinNode(int local)
{
    return pin_state.lease(local);
}

bool Tile::isPinNodeLeased(int local) const
{
    return (pin_state.leased_nodes & (NodeMask{0,1} << local)) != NodeMask{};
}

int Tile::getNodeNum(std::string type, std::string port, int pos)
{
    // A generic inverter occupies the primary LUT element and its I0/O pins;
    // this is a primitive-model alias, not device knowledge.
    if (type == "INV") {
        type = "LUT6";
        if (port == "I") {
            port = "I0";
        }
    }
    int bit = extractIndexedPort(port);
    if (type.find("LUT") == 0 && bit >= 0 && port == "I") {
        port = "I" + std::to_string(bit);
    }
    if (type.find("CARRY") == 0 && bit >= 0) {
        if (port == "DI" || port == "S" || port == "O") {
            port += std::to_string(bit);
        }
        else if (port == "CO") {
            port = "C" + std::to_string(bit);
        }
    }
    if (type.find("MUX") == 0 && bit >= 0 && port == "I") {
        port = "I" + std::to_string(bit);
    }

    static constexpr int lut_out[4] = {16, 80, 144, 212};
    static constexpr int lut_in0[4] = {17, 81, 145, 213};
    static constexpr int lut_in1[4] = {18, 82, 146, 214};
    static constexpr int lut_in2[4] = {19, 83, 147, 215};
    static constexpr int lut_in3[4] = {20, 84, 148, 216};
    static constexpr int lut_in4[4] = {21, 85, 149, 217};
    static constexpr int lut_in5[4] = {22, 86, 150, 218};
    static constexpr int ff_d[4] = {31, 95, 130, 198};
    static constexpr int ff_q[4] = {1, 65, 129, 197};
    static constexpr int mux_out[4] = {17, 81, 145, 213};

    if (type.find("FD") == 0) {
        int bel = belIndexFromPlacedPos(pos);
        if (port == "C") return 0;
        if (port == "CE" || port == "EN") return 1;
        if (port == "D") return indexedNode(ff_d, bel);
        if (port == "Q") return indexedNode(ff_q, bel);
        if (port == "R" || port == "S" || port == "CLR" || port == "PRE" || port == "SRST" || port == "ARST") return 199;
    }
    if (type.find("LUT") == 0) {
        int bel = belIndexFromPlacedPos(pos);
        if (port == "I0" || port == "A1") return indexedNode(lut_in0, bel);
        if (port == "I1" || port == "A2") return indexedNode(lut_in1, bel);
        if (port == "I2" || port == "A3") return indexedNode(lut_in2, bel);
        if (port == "I3" || port == "A4") return indexedNode(lut_in3, bel);
        if (port == "I4" || port == "A5") return indexedNode(lut_in4, bel);
        if (port == "I5" || port == "A6") return indexedNode(lut_in5, bel);
        if (type == "LUT1" && port == "O") return indexedNode(mux_out, bel);
        if (port == "O6" || port == "O") return indexedNode(lut_out, bel);
        if (port == "O5") return indexedNode(ff_d, bel);
        if (port == "WCLK") return 0;
        if (port == "WE") return 39;
    }
    if (type.find("CARRY") == 0) {
        int bel = belIndexFromBitOrPos(bit, pos);
        if (port == "CI" || port == "CYINIT") return 9;
        if (port == "DI0" || port == "DI1" || port == "DI2" || port == "DI3") return indexedNode(ff_d, bel);
        if (port == "S0" || port == "S1" || port == "S2" || port == "S3") return indexedNode(lut_in0, bel);
        if (port == "C0" || port == "C1" || port == "C2") return indexedNode(ff_d, bel);
        if (port == "C3") return 63;
        if (port == "O0" || port == "O1" || port == "O2" || port == "O3") return indexedNode(mux_out, bel);
    }
    if (type.find("MUX") == 0) {
        if (port == "I0" || port == "I1") return indexedNode(mux_out, muxDataBelFromPlacedPos(type, port, pos));
        if (port == "O") return indexedNode(mux_out, muxOutputBelFromPlacedPos(type, pos));
        if (port == "S") return indexedNode(ff_d, muxControlBelFromPlacedPos(type, pos));
    }
    return -1;
}

bool fpga::isPlaceableElement(const rtl::Inst& inst)
{
    return maybeInstElementType(inst).has_value();
}

std::optional<ElementType> fpga::elementTypeForInst(const rtl::Inst& inst)
{
    // Expose the abstract element class so placement can skip incompatible tiles.
    return maybeInstElementType(inst);
}

void Tile::assign(rtl::Inst* inst)
{
    PNR_ASSERT(inst->tile.peer == nullptr, "assigning tile {} to already assigned inst {}", makeName(), inst->makeName(), inst->tile->makeName());
    inst->tile.set(static_cast<Referable<Tile>*>(this));
    invalidatePlacementCaches();
}

void Tile::invalidatePlacementCaches()
{
    // Element occupancy is local, while input-joint ownership is shared by
    // every resource tile attached to the same physical crossbar.
    elements_initialized = false;
    for (Tile* attached : attachedResourceTiles(*this)) {
        attached->input_joint_reservations_initialized = false;
    }
}

bool Tile::unassign(rtl::Inst* inst)
{
    // Drop the element counters and invalidate masks so the remaining peers rebuild them exactly.
    if (!inst || inst->tile.peer != this || !inst->cell_ref.peer) {
        return false;
    }
    switch (instElementType(*inst)) {
    case ELEMENT_FD:
        regs_cnt = std::max(0, regs_cnt - 1);
        break;
    case ELEMENT_LUT1:
        luts1cnt = std::max(0, luts1cnt - 1);
        break;
    case ELEMENT_LUT5:
        if (inst->cnt_inputs == 6) luts6cnt = std::max(0, luts6cnt - 1);
        else luts5cnt = std::max(0, luts5cnt - 1);
        break;
    case ELEMENT_CARRY:
        carry = std::max(0, carry - 4);
        break;
    case ELEMENT_MUXF7:
    case ELEMENT_MUXF8:
        mux = std::max(0, mux - 1);
        break;
    default:
        break;
    }
    inst->tile.clear();
    inst->pos = -1;
    invalidatePlacementCaches();
    return true;
}

bool Tile::hasFreeElement(ElementType type)
{
    // Rebuild occupancy lazily and answer only the monotonic capacity question.
    ensureElementState(*this);
    return tile_type && elements_initialized && elements_free[type] != 0;
}

bool Tile::hasOccupiedElementNeighbors(rtl::Inst* inst)
{
    // A timing move may only detach an element with no occupied resource-chain
    // neighbors; linked packed elements must be moved by a dedicated packer.
    if (!inst || inst->tile.peer != this || inst->pos < 0) {
        return false;
    }
    ensureElementState(*this);
    ElementType type = instElementType(*inst);
    int bit = elementBitFromPlacedPos(type, inst->pos);
    if (bit < 0 || bit >= ELEMENT_BITMAP_BITS) {
        return false;
    }
    std::array<std::array<bool, ELEMENT_BITMAP_BITS>, ELEMENT_TYPE_COUNT> left_visited{};
    std::array<std::array<bool, ELEMENT_BITMAP_BITS>, ELEMENT_TYPE_COUNT> right_visited{};
    return linkedElementStatus(*this, inst, type, bit, true, left_visited)
            != BlockerStatus::clear
        || linkedElementStatus(*this, inst, type, bit, false, right_visited)
            != BlockerStatus::clear;
}

int Tile::peekAdd(rtl::Inst* inst, bool enforce_route_capacity)
{
    // Run the exact selector but deliberately omit assignment, counters,
    // resource masks and route-side effects.
    if (!inst || !inst->cell_ref.peer || inst->tile.peer) {
        return -1;
    }
    int pos = -1;
    bool previous_route_capacity = enforce_pack_route_capacity;
    enforce_pack_route_capacity = enforce_route_capacity;
    bool placement_ok = tryElementPlacement(
        *this, inst, instElementType(*inst), pos);
    enforce_pack_route_capacity = previous_route_capacity;
    return placement_ok ? pos : -1;
}

struct fpga::ElementPackingPreview::Impl
{
    struct Reservation
    {
        rtl::Inst* inst = nullptr;
        Referable<Tile>* original_tile = nullptr;
        Coord original_coord{-1, -1};
        int original_pos = -1;
        std::array<uint16_t, ELEMENT_TYPE_COUNT> free_before{};
        int pos = -1;
    };

    Tile& tile;
    std::array<uint16_t, ELEMENT_TYPE_COUNT> original_pos{};
    std::array<uint16_t, ELEMENT_TYPE_COUNT> original_free{};
    std::array<std::array<uint16_t, ELEMENT_BITMAP_BITS>, ELEMENT_TYPE_COUNT>
        original_left{};
    std::array<std::array<uint16_t, ELEMENT_BITMAP_BITS>, ELEMENT_TYPE_COUNT>
        original_right{};
    bool original_initialized = false;
    struct InputReservationSnapshot
    {
        Tile* tile = nullptr;
        std::unordered_map<int, rtl::Conn*> local;
        std::vector<std::pair<rtl::Conn*, NodeMask>> joints;
        bool initialized = false;
    };
    std::vector<InputReservationSnapshot> input_reservations;
    std::vector<Reservation> reservations;

    explicit Impl(Tile& selected_tile) : tile(selected_tile)
    {
        original_pos = tile.elements_pos;
        original_free = tile.elements_free;
        original_left = tile.elements_left;
        original_right = tile.elements_right;
        original_initialized = tile.elements_initialized;
        ensureElementState(tile);
        for (Tile* route_tile : attachedResourceTiles(tile)) {
            input_reservations.push_back({
                .tile = route_tile,
                .local = route_tile->input_local_reservations,
                .joints = route_tile->input_joint_reservations,
                .initialized =
                    route_tile->input_joint_reservations_initialized,
            });
        }
    }

    void invalidateInputReservations()
    {
        for (InputReservationSnapshot& snapshot : input_reservations) {
            snapshot.tile->input_joint_reservations_initialized = false;
        }
    }

    void restoreOriginalState()
    {
        tile.elements_pos = original_pos;
        tile.elements_free = original_free;
        tile.elements_left = original_left;
        tile.elements_right = original_right;
        tile.elements_initialized = original_initialized;
        for (InputReservationSnapshot& snapshot : input_reservations) {
            snapshot.tile->input_local_reservations = snapshot.local;
            snapshot.tile->input_joint_reservations = snapshot.joints;
            snapshot.tile->input_joint_reservations_initialized =
                snapshot.initialized;
        }
    }
};

ElementPackingPreview::ElementPackingPreview(Tile& tile)
    : impl(std::make_unique<Impl>(tile))
{
}

ElementPackingPreview::~ElementPackingPreview()
{
    rollback(0);
    impl->restoreOriginalState();
}

int ElementPackingPreview::peek(rtl::Inst* inst,
                                bool enforce_route_capacity)
{
    return impl->tile.peekAdd(inst, enforce_route_capacity);
}

int ElementPackingPreview::reserveAt(rtl::Inst* inst, int pos,
                                     bool enforce_route_capacity)
{
    if (!inst || !inst->cell_ref.peer || inst->tile.peer) {
        return -1;
    }
    ElementType type = instElementType(*inst);
    bool previous_route_capacity = enforce_pack_route_capacity;
    enforce_pack_route_capacity = enforce_route_capacity;
    bool placement_ok = elementPlacementAtLegal(
        impl->tile, inst, type, pos);
    enforce_pack_route_capacity = previous_route_capacity;
    if (!placement_ok) {
        return -1;
    }

    Impl::Reservation reservation{
        .inst = inst,
        .original_tile = inst->tile.peer,
        .original_coord = inst->coord,
        .original_pos = inst->pos,
        .free_before = impl->tile.elements_free,
        .pos = pos,
    };
    inst->pos = pos;
    inst->coord = impl->tile.coord;
    inst->tile.set(static_cast<Referable<Tile>*>(&impl->tile));
    int bit = elementBitFromPlacedPos(type, pos);
    reserveElementBit(impl->tile, type, bit, inst);
    impl->tile.elements_initialized = true;
    impl->reservations.push_back(reservation);
    impl->invalidateInputReservations();
    return pos;
}

int ElementPackingPreview::reserve(rtl::Inst* inst,
                                   bool enforce_route_capacity)
{
    int pos = peek(inst, enforce_route_capacity);
    return pos >= 0 ? reserveAt(inst, pos, enforce_route_capacity) : -1;
}

size_t ElementPackingPreview::checkpoint() const
{
    return impl->reservations.size();
}

void ElementPackingPreview::rollback(size_t checkpoint)
{
    PNR_ASSERT(checkpoint <= impl->reservations.size(),
        "invalid Element packing preview checkpoint");
    while (impl->reservations.size() > checkpoint) {
        Impl::Reservation reservation = impl->reservations.back();
        impl->reservations.pop_back();
        reservation.inst->tile.clear();
        if (reservation.original_tile) {
            reservation.inst->tile.set(reservation.original_tile);
        }
        reservation.inst->coord = reservation.original_coord;
        reservation.inst->pos = reservation.original_pos;
        impl->tile.elements_free = reservation.free_before;
        impl->tile.elements_initialized = true;
        impl->invalidateInputReservations();
    }
}

bool ElementPackingPreview::reservePack(
    const std::vector<rtl::Inst*>& insts,
    std::vector<ElementPackingChoice>& choices,
    bool enforce_route_capacity)
{
    size_t start = checkpoint();
    std::vector<bool> selected(insts.size());
    std::function<bool(size_t)> recurse = [&](size_t placed) {
        if (placed == insts.size()) {
            return true;
        }
        for (size_t index = 0; index < insts.size(); ++index) {
            rtl::Inst* inst = insts[index];
            if (selected[index] || !inst || inst->tile.peer) continue;
            std::vector<int> positions = impl->tile.candidatePositions(inst);
            for (int pos : positions) {
                size_t branch = checkpoint();
                if (reserveAt(inst, pos, enforce_route_capacity) < 0) {
                    continue;
                }
                selected[index] = true;
                if (recurse(placed + 1)) {
                    return true;
                }
                selected[index] = false;
                rollback(branch);
            }
        }
        return false;
    };
    if (!recurse(0)) {
        rollback(start);
        return false;
    }
    for (size_t index = start; index < impl->reservations.size(); ++index) {
        const Impl::Reservation& reservation = impl->reservations[index];
        choices.push_back(ElementPackingChoice{
            .inst = reservation.inst,
            .pos = reservation.pos,
        });
    }
    return true;
}

int Tile::tryAdd(rtl::Inst* inst, bool enforce_route_capacity)  // it's not SRL
{
    PNR_ASSERT(coord.x > -1 && coord.y > -1, "trying to add inst '{}' to a tile '{}' with coords -1", inst->makeName(), makeName());
    if (!inst->cell_ref.peer) {
        return -1;
    }
    ElementType type = instElementType(*inst);
    int pos = -1;
    bool previous_route_capacity = enforce_pack_route_capacity;
    enforce_pack_route_capacity = enforce_route_capacity;
    bool placement_ok = tryElementPlacement(*this, inst, type, pos);
    enforce_pack_route_capacity = previous_route_capacity;
    if (!placement_ok) {
        return -1;
    }

    switch (type) {
    case ELEMENT_FD:
        ++regs_cnt;
        break;
    case ELEMENT_LUT1:
        ++luts1cnt;
        break;
    case ELEMENT_LUT5:
        if (inst->cnt_inputs == 6) ++luts6cnt;
        else ++luts5cnt;
        break;
    case ELEMENT_CARRY:
        carry += 4;
        break;
    case ELEMENT_MUXF7:
    case ELEMENT_MUXF8:
        ++mux;
        break;
    default:
        break;
    }

    inst->pos = pos;
    inst->coord = coord;
    assign(inst);
    int bit = elementBitFromPlacedPos(type, pos);
    reserveElementBit(*this, type, bit, inst);
    elements_initialized = true;
    markVoidNetsForTile(*this);
    if (packDebugEnabled()) {
        std::fprintf(stderr, "pack-debug commit inst=%s element=%s tile=%s pos=%d bit=%d\n",
            inst->makeName().c_str(), elementTypeName(type), makeName().c_str(), pos, bit);
        printTypeMasks("pack-debug   free-after", elements_free);
    }
    return pos;
}

int Tile::tryAddAt(rtl::Inst* inst, int pos, bool enforce_route_capacity)
{
    // Place at a caller-selected element position while preserving tryAdd checks.
    PackDebugScope debug_scope(inst);
    struct RouteCapacityRestore
    {
        bool previous = enforce_pack_route_capacity;
        ~RouteCapacityRestore() { enforce_pack_route_capacity = previous; }
    } restore_route_capacity;
    enforce_pack_route_capacity = enforce_route_capacity;
    PNR_ASSERT(coord.x > -1 && coord.y > -1, "trying to add inst '{}' to a tile '{}' with coords -1", inst->makeName(), makeName());
    if (!inst->cell_ref.peer) {
        return -1;
    }
    ElementType type = instElementType(*inst);
    ensureElementState(*this);
    int bit = elementBitFromPlacedPos(type, pos);
    if (bit < 0 || bit >= ELEMENT_BITMAP_BITS) {
        if (packDebugEnabled()) {
            std::fprintf(stderr, "pack-debug try-at reject inst=%s tile=%s pos=%d bit=%d reason=bad-bit\n",
                inst->makeName().c_str(), makeName().c_str(), pos, bit);
        }
        return -1;
    }
    if ((elements_pos[type] & bit16(bit)) == 0) {
        if (packDebugEnabled()) {
            std::fprintf(stderr, "pack-debug try-at reject inst=%s element=%s tile=%s full=%s type=%s pos=%d bit=%d reason=no-element posmask=0x%04x free=0x%04x\n",
                inst->makeName().c_str(), elementTypeName(type), makeName().c_str(), full_name.c_str(),
                tile_type ? tile_type->name.c_str() : "", pos, bit, elements_pos[type], elements_free[type]);
        }
        return -1;
    }
    if ((elements_free[type] & bit16(bit)) == 0) {
        if (packDebugEnabled()) {
            rtl::Inst* owner = elementInstAt(*this, type, bit);
            std::fprintf(stderr, "pack-debug try-at reject inst=%s element=%s tile=%s full=%s type=%s pos=%d bit=%d reason=busy owner=%s posmask=0x%04x free=0x%04x\n",
                inst->makeName().c_str(), elementTypeName(type), makeName().c_str(), full_name.c_str(),
                tile_type ? tile_type->name.c_str() : "", pos, bit,
                owner ? owner->makeName().c_str() : "", elements_pos[type], elements_free[type]);
        }
        return -1;
    }
    if (!canHost(*this, inst, pos)) {
        if (packDebugEnabled()) {
            std::fprintf(stderr, "pack-debug try-at reject inst=%s cell=%s element=%s tile=%s full=%s type=%s pos=%d bit=%d reason=host posmask=0x%04x free=0x%04x\n",
                inst->makeName().c_str(), inst->cell_ref.peer ? inst->cell_ref->type.c_str() : "",
                elementTypeName(type), makeName().c_str(), full_name.c_str(),
                tile_type ? tile_type->name.c_str() : "", pos, bit, elements_pos[type], elements_free[type]);
        }
        return -1;
    }
    if (!neighborsCompatible(*this, inst, type, bit)) {
        if (packDebugEnabled()) {
            std::fprintf(stderr, "pack-debug try-at reject inst=%s cell=%s element=%s tile=%s full=%s type=%s pos=%d bit=%d reason=chain posmask=0x%04x free=0x%04x\n",
                inst->makeName().c_str(), inst->cell_ref.peer ? inst->cell_ref->type.c_str() : "",
                elementTypeName(type), makeName().c_str(), full_name.c_str(),
                tile_type ? tile_type->name.c_str() : "", pos, bit, elements_pos[type], elements_free[type]);
        }
        return -1;
    }

    switch (type) {
    case ELEMENT_FD:
        ++regs_cnt;
        break;
    case ELEMENT_LUT1:
        ++luts1cnt;
        break;
    case ELEMENT_LUT5:
        if (inst->cnt_inputs == 6) ++luts6cnt;
        else ++luts5cnt;
        break;
    case ELEMENT_CARRY:
        carry += 4;
        break;
    case ELEMENT_MUXF7:
    case ELEMENT_MUXF8:
        ++mux;
        break;
    default:
        break;
    }

    inst->pos = pos;
    inst->coord = coord;
    assign(inst);
    reserveElementBit(*this, type, bit, inst);
    markVoidNetsForTile(*this);
    if (packDebugEnabled()) {
        std::fprintf(stderr, "pack-debug commit-at inst=%s element=%s tile=%s pos=%d bit=%d\n",
            inst->makeName().c_str(), elementTypeName(type), makeName().c_str(), pos, bit);
        printTypeMasks("pack-debug   free-after", elements_free);
    }
    return pos;
}

std::vector<int> Tile::candidatePositions(rtl::Inst* inst)
{
    // Enumerate only canonical positions backed by this tile's element bits.
    std::vector<int> positions;
    if (!inst || !inst->cell_ref.peer) {
        return positions;
    }
    ensureElementState(*this);
    ElementType type = instElementType(*inst);
    uint16_t existing = elements_pos[type];
    while (existing) {
        int bit = std::countr_zero(static_cast<unsigned>(existing));
        existing &= static_cast<uint16_t>(existing - 1);
        int pos = placedPosFromElementBit(type, bit);
        if (pos >= 0) {
            positions.push_back(pos);
        }
    }
    return positions;
}
