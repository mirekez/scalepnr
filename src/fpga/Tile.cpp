#include "Tile.h"
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
#include <memory>
#include <optional>
#include <utility>

using namespace fpga;

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
    if (type == "LUT1" || inst.cnt_inputs == 1) {
        return ELEMENT_LUT1;
    }
    return ELEMENT_LUT5;
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

bool packDebugEnabled()
{
    return std::getenv("SCALEPNR_PACK_DEBUG") != nullptr;
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
    // Classify LUT primitives by generic cell type prefix.
    return inst.cell_ref.peer && inst.cell_ref.peer->type.find("LUT") == 0;
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

rtl::Net* findNetByDesignator(rtl::Inst& inst, int designator)
{
    // Find the flat net attached to a local connection designator.
    rtl::Module* parent = parentModule(inst);
    if (!parent) {
        return nullptr;
    }
    for (auto& net : parent->nets) {
        for (int net_designator : net.designators) {
            if (net_designator == designator) {
                return &net;
            }
        }
    }
    return nullptr;
}

bool canHost(Tile& tile, rtl::Inst* inst, int pos);

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
        if (conn.port_ref.peer && conn.port_ref->type == rtl::Port::PORT_OUT) {
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
    return net && !net->void_net;
}

bool connHasExternalNet(rtl::Inst& inst, rtl::Conn& conn)
{
    // A non-void input net means this pin still needs an exclusive fabric local.
    if (!conn.port_ref.peer || conn.port_ref->designator < 0) {
        return false;
    }
    rtl::Net* net = findNetByDesignator(inst, conn.port_ref->designator);
    return net && !net->void_net;
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
    default: return {"I0", "O"};
    }
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
    static int next_designator = 100000000;
    for (const auto& net : module.nets) {
        for (int designator : net.designators) {
            next_designator = std::max(next_designator, designator + 1);
        }
    }
    return next_designator++;
}

rtl::Net* findNetInModuleByDesignator(rtl::Module& module, int designator)
{
    for (auto& net : module.nets) {
        if (std::find(net.designators.begin(), net.designators.end(), designator) != net.designators.end()) {
            return &net;
        }
    }
    return nullptr;
}

rtl::Net* appendGeneratedNet(rtl::Module& module, const std::string& name, int designator, bool void_net)
{
    auto& net = module.nets.emplace_back();
    net.name = void_net ? std::string("void") : name;
    net.designators.push_back(designator);
    net.void_net = void_net;
    return &net;
}

Referable<rtl::Cell>* makeGeneratedCell(rtl::Inst& near_inst, ElementType type)
{
    static int generated_cell_index = 0;
    auto* cell = new Referable<rtl::Cell>();
    auto [input_port, output_port] = passthroughPorts(type);
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

rtl::Inst* makeGeneratedPassthroughInst(rtl::Inst& near_inst, ElementType type)
{
    static int generated_inst_index = 0;
    Referable<rtl::Inst>* parent = near_inst.parent_ref.peer;
    PNR_ASSERT(parent, "cannot insert passthrough for '{}' without a parent instance", near_inst.makeName());

    auto& inst = parent->insts.emplace_back();
    inst.cell_ref.set(makeGeneratedCell(near_inst, type));
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
    for (auto& conn : sink.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        rtl::Conn* followed = conn.follow();
        if (!followed || followed->inst_ref.peer != &driver || !followed->port_ref.peer
            || followed->port_ref->type != rtl::Port::PORT_OUT) {
            continue;
        }
        if (rtl::Net* net = findNetByDesignator(sink, conn.port_ref->designator)) {
            if (!net->void_net) {
                fpga::unrouteNet(*net);
                net->void_net = true;
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
                        if (std::getenv("SCALEPNR_PACK_DEBUG")) {
                            std::fprintf(stderr, "pack debug: left blocker %s bit %d (%s) is not connected to %s bit %d (%s)\n",
                                elementTypeName(neighbor_type), neighbor_bit, neighbor->makeName().c_str(),
                                elementTypeName(type), bit, inst->makeName().c_str());
                        }
                        return BlockerStatus::incompatible;
                    }
                    if (neighbor_type == ELEMENT_LUT1 && type == ELEMENT_CARRY
                        && !outputOnlyDrives(*neighbor, *inst)) {
                        if (std::getenv("SCALEPNR_PACK_DEBUG")) {
                            std::fprintf(stderr, "pack debug: left blocker %s bit %d (%s) has external output users\n",
                                elementTypeName(neighbor_type), neighbor_bit, neighbor->makeName().c_str());
                        }
                        return BlockerStatus::incompatible;
                    }
                }
                else if (!connectedInOrder(*inst, *neighbor)) {
                    if (std::getenv("SCALEPNR_PACK_DEBUG")) {
                        std::fprintf(stderr, "pack debug: %s bit %d (%s) is not connected to right blocker %s bit %d (%s)\n",
                            elementTypeName(type), bit, inst->makeName().c_str(),
                            elementTypeName(neighbor_type), neighbor_bit, neighbor->makeName().c_str());
                    }
                    return BlockerStatus::incompatible;
                }
                else if (type == ELEMENT_LUT1 && neighbor_type == ELEMENT_CARRY
                    && !outputOnlyDrives(*inst, *neighbor)) {
                    if (std::getenv("SCALEPNR_PACK_DEBUG")) {
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

u256 outputNodesForElement(Tile& tile, ElementType type, int bit, rtl::Inst* inst)
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
    if (!inst || !inst->cell_ref.peer || !inst->cell_ref->attributes.contains("scalepnr_passthrough")) {
        return true;
    }
    u256 candidate_nodes = outputNodesForElement(tile, type, bit, inst);
    if (candidate_nodes == u256{}) {
        return true;
    }
    bool candidate_external = inst && hasExternalOutputNet(*inst);
    for (rtl::Inst* owner : assignedInsts(tile)) {
        if (!owner || owner == inst || !owner->cell_ref.peer) {
            continue;
        }
        ElementType owner_type = instElementType(*owner);
        int owner_bit = elementBitFromPlacedPos(owner_type, owner->pos);
        u256 owner_nodes = outputNodesForElement(tile, owner_type, owner_bit, owner);
        if ((candidate_nodes & owner_nodes) == u256{}) {
            continue;
        }
        if (connectedInOrder(*owner, *inst) && outputOnlyDrives(*owner, *inst)) {
            continue;
        }
        if (connectedInOrder(*inst, *owner) && outputOnlyDrives(*inst, *owner)) {
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
        for (const auto& entry : tile.tile_type->pin_map.resource_pin_names) {
            if (entry.first.type != static_cast<uint8_t>(TILE_PIN_INPUT) || entry.second != pin) {
                continue;
            }
            if (site_pos >= 0 && entry.first.value / 256 != site_pos) {
                continue;
            }
            resources.push_back(entry.first.value);
        }
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
        u256 nodes = tile.tile_type->pin_map.getInputNodes(resource_node);
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
        u256 nodes = tile.getPinNodes(type, port, pos);
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

u256 inputNodesForInstAt(Tile& tile, rtl::Inst& inst, int pos, bool external_only)
{
    // Collect local input nodes used by routed input pins of one placed instance.
    if (!inst.cell_ref.peer || pos < 0) {
        return {};
    }
    u256 nodes{};
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

u256 inputNodesForInst(Tile& tile, rtl::Inst& inst, bool external_only)
{
    return inputNodesForInstAt(tile, inst, inst.pos, external_only);
}

u256 inputNodesForElement(Tile& tile, ElementType type, int bit, rtl::Inst* inst)
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

bool generatedPassthroughInputNeedsFabric(rtl::Inst& inst)
{
    // Source passthrough inputs are void tile-local; target passthrough inputs are routed.
    if (!inst.cell_ref.peer || !inst.cell_ref->attributes.contains("scalepnr_passthrough")) {
        return false;
    }
    const std::string& kind = inst.cell_ref->attributes["scalepnr_passthrough"];
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
    if (inst->cell_ref->attributes.contains("scalepnr_passthrough")
        && !generatedPassthroughInputNeedsFabric(*inst)) {
        return true;
    }
    u256 candidate_nodes = inputNodesForElement(tile, type, bit, inst);
    if (candidate_nodes == u256{}) {
        return true;
    }
    std::vector<InputRouteEndpoint> candidate_endpoints;
    if (inst && inst->cell_ref.peer) {
        int pos = placedPosFromElementBit(type, bit);
        candidate_endpoints = inputRouteEndpointsForInstAt(tile, *inst, pos, true);
    }
    for (rtl::Inst* owner : assignedInsts(tile)) {
        if (!owner || owner == inst || !owner->cell_ref.peer) {
            continue;
        }
        u256 owner_nodes = inputNodesForInst(tile, *owner, true);
        bool endpoint_conflict = (candidate_nodes & owner_nodes) != u256{};
        if (!candidate_endpoints.empty()) {
            endpoint_conflict = false;
            std::vector<InputRouteEndpoint> owner_endpoints =
                inputRouteEndpointsForInstAt(tile, *owner, owner->pos, true);
            for (const InputRouteEndpoint& candidate_endpoint : candidate_endpoints) {
                for (const InputRouteEndpoint& owner_endpoint : owner_endpoints) {
                    if (sameInputRouteEndpoint(candidate_endpoint, owner_endpoint)) {
                        endpoint_conflict = true;
                        break;
                    }
                }
                if (endpoint_conflict) {
                    break;
                }
            }
        }
        if (!endpoint_conflict) {
            continue;
        }
        if (packDebugEnabled()) {
            std::fprintf(stderr, "pack-debug   reject bit=%d reason=input-alias inst=%s owner=%s nodes=%s\n",
                bit, inst->makeName().c_str(), owner->makeName().c_str(),
                (candidate_nodes & owner_nodes).str().c_str());
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
            bool inst_passthrough = inst && inst->cell_ref.peer
                && inst->cell_ref->attributes.contains("scalepnr_passthrough");
            bool paired_passthrough = paired->cell_ref.peer
                && paired->cell_ref->attributes.contains("scalepnr_passthrough");
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

bool placeGeneratedAtElement(Tile& tile, rtl::Inst& inst, ElementType type, int bit)
{
    // Commit a generated passthrough into the exact linked element position.
    ensureElementState(tile);
    if (bit < 0 || bit >= ELEMENT_BITMAP_BITS || (tile.elements_free[type] & bit16(bit)) == 0) {
        return false;
    }
    int pos = placedPosFromElementBit(type, bit);
    if (pos < 0 || !canHost(tile, &inst, pos) || !neighborsCompatible(tile, &inst, type, bit)) {
        return false;
    }
    inst.pos = pos;
    inst.coord = tile.coord;
    tile.assign(&inst);
    reserveElementBit(tile, type, bit, &inst);
    return true;
}

struct NeighborElement
{
    ElementType type = ELEMENT_LUT5;
    int bit = -1;
};

std::optional<NeighborElement> firstFreeNeighbor(Tile& tile, ElementType type, int bit, bool right_side)
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
            if (elementColumn(neighbor_type) != target_column) {
                continue;
            }
            for (int neighbor_bit = 0; neighbor_bit < ELEMENT_BITMAP_BITS; ++neighbor_bit) {
                if ((links[neighbor_bit] & bit16(bit)) == 0) {
                    continue;
                }
                if (right_side && neighbor_type == ELEMENT_FD) {
                    continue;
                }
                if ((tile.elements_free[neighbor_type] & bit16(neighbor_bit)) != 0) {
                    int pos = placedPosFromElementBit(neighbor_type, neighbor_bit);
                    auto [input_port, output_port] = passthroughPorts(neighbor_type);
                    u256 route_nodes = right_side
                        ? tile.getOutputPinNodes(passthroughCellType(neighbor_type), output_port, pos)
                        : tile.getPinNodes(passthroughCellType(neighbor_type), input_port, pos);
                    if (route_nodes != u256{} && (route_nodes & tile.pin_state.leased_nodes) != u256{}) {
                        continue;
                    }
                    return NeighborElement{neighbor_type, neighbor_bit};
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

void connectConns(rtl::Conn& input, rtl::Conn& output, int designator)
{
    input.port_ref->designator = designator;
    output.port_ref->designator = designator;
    input.set(&rtl::Conn::fromBase(output));
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

    std::vector<RefBase<Referable<rtl::Conn>>*> old_sinks = rtl::Conn::getSinks(*source_out);
    for (auto* sink_ref : old_sinks) {
        rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
        if (!sink_conn || !sink_conn->inst_ref.peer || !sink_conn->inst_ref->cell_ref.peer) {
            continue;
        }
        if (sink_conn->inst_ref->cell_ref->attributes["scalepnr_passthrough"] == "source") {
            rtl::Conn* pass_out = firstOutputConn(*sink_conn->inst_ref.peer);
            if (pass_out && pass_out->port_ref.peer) {
                from = sink_conn->inst_ref.peer;
                from_port = pass_out->port_ref->makeName();
                if (rtl::Module* module = ownerModule(*from)) {
                    net = findNetInModuleByDesignator(*module, pass_out->port_ref->designator);
                }
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

    rtl::Inst* pass = makeGeneratedPassthroughInst(*from, neighbor->type);
    pass->cell_ref->attributes["scalepnr_passthrough"] = "source";
    rtl::Conn* pass_in = firstInputConn(*pass);
    rtl::Conn* pass_out = firstOutputConn(*pass);
    PNR_ASSERT(pass_in && pass_out, "generated source passthrough is missing ports");

    connectConns(*pass_in, *source_out, void_designator);
    pass_out->port_ref->designator = route_designator;
    for (auto* sink_ref : old_sinks) {
        rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
        if (sink_conn && sink_conn != pass_in) {
            sink_conn->set(&rtl::Conn::fromBase(*pass_out));
        }
    }
    if (!placeGeneratedAtElement(tile, *pass, neighbor->type, neighbor->bit)) {
        source_out->port_ref->designator = route_designator;
        for (auto* sink_ref : old_sinks) {
            rtl::Conn* sink_conn = sink_ref ? rtl::Conn::fromBase(sink_ref) : nullptr;
            if (sink_conn && sink_conn != pass_in) {
                sink_conn->set(&rtl::Conn::fromBase(*source_out));
            }
        }
        if (pass->parent_ref.peer && !pass->parent_ref->insts.empty()
            && &pass->parent_ref->insts.back() == pass) {
            pass->parent_ref->insts.pop_back();
        }
        return false;
    }

    // The route net changes physical source from the original output to this
    // passthrough output; stale routes from the old output must release leases.
    fpga::unrouteNet(*route_net);

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
    if (!isTargetPassthroughPort(type, to_port)) {
        return false;
    }
    int bit = elementBitFromPlacedPos(type, to->pos);
    if (bit < 0 || !hasNeighbor(tile, type, bit, false)) {
        return false;
    }

    rtl::Conn* target_in = findConn(*to, to_port, rtl::Port::PORT_IN);
    if (!target_in) {
        target_in = firstInputConn(*to);
    }
    if (!target_in || !target_in->port_ref.peer) {
        return false;
    }
    rtl::Conn* driver = target_in->follow();
    if (!driver || !driver->port_ref.peer) {
        return false;
    }
    if (driver->inst_ref.peer && driver->inst_ref->cell_ref.peer
        && driver->inst_ref->cell_ref->attributes["scalepnr_passthrough"] == "target") {
        rtl::Inst* pass = driver->inst_ref.peer;
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

    std::optional<NeighborElement> neighbor = firstFreeNeighbor(tile, type, bit, false);
    if (!neighbor) {
        return false;
    }
    rtl::Module* module = ownerModule(*to);
    if (!module) {
        return false;
    }
    int route_designator = target_in->port_ref->designator;
    if (route_designator < 0) {
        return false;
    }
    int void_designator = nextGeneratedDesignator(*module);
    rtl::Net* route_net = findNetInModuleByDesignator(*module, route_designator);
    if (!route_net) {
        return false;
    }

    rtl::Inst* pass = makeGeneratedPassthroughInst(*to, neighbor->type);
    pass->cell_ref->attributes["scalepnr_passthrough"] = "target";
    rtl::Conn* pass_in = firstInputConn(*pass);
    rtl::Conn* pass_out = firstOutputConn(*pass);
    PNR_ASSERT(pass_in && pass_out, "generated target passthrough is missing ports");

    connectConns(*pass_in, *driver, route_designator);
    connectConns(*target_in, *pass_out, void_designator);
    if (!placeGeneratedAtElement(tile, *pass, neighbor->type, neighbor->bit)) {
        target_in->port_ref->designator = route_designator;
        target_in->set(&rtl::Conn::fromBase(*driver));
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

    const std::string& type = inst->cell_ref->type;
    if (type.find("FD") == 0 || type.find("LUT") == 0 || type.find("CARRY") == 0 || type.find("MUX") == 0) {
        return tileTypeHasLogicElements(*tile.tile_type) && carryLutSlotCompatible(tile, inst, pos);
    }
    return true;
}

bool useResourcePinNameFallback(const std::string& type)
{
    // Restrict resource-pin-name fallback to logic primitives with packed site pins.
    return type.find("LUT") == 0
        || type.find("FD") == 0
        || type.find("CARRY") == 0
        || type.find("MUX") == 0
        || type == "IBUF"
        || type == "OBUF";
}

std::string normalizedResourcePinName(std::string type, std::string port, int pos)
{
    // Convert generic cell ports and placement position to a tile resource pin name.
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
        bel = muxDataBelFromPlacedPos(type, port == "O" ? "I0" : port, pos);
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

bool fpga::preparePassthroughRouteEndpoints(rtl::Inst*& from, std::string& from_port,
                                            rtl::Inst*& to, std::string& to_port,
                                            rtl::Net*& net, bool allow_new_source_passthrough)
{
    bool changed = false;
    if (ensureSourcePassthrough(from, from_port, net, allow_new_source_passthrough)) {
        changed = true;
    }
    if (ensureTargetPassthrough(to, to_port, net)) {
        changed = true;
    }
    return changed;
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

u256 Tile::getPinNodes(const std::string& type, const std::string& port, int pos) const
{
    if (type == "OBUF" && port == "I" && cb_type) {
        u256 nodes{};
        for (const char* name : {"IMUX34", "IMUX_L34", "IMUX_R34", "IMUX18", "IMUX_L18", "IMUX_R18"}) {
            int node = cb_type->localNodeNum(name);
            if (node >= 0) {
                cb_type->rememberNodeName(CB_NODE_LOCAL, node, name);
                nodes |= u256{0,1} << node;
            }
        }
        if (nodes != u256{}) {
            return nodes;
        }
    }

    int local = const_cast<Tile*>(this)->getNodeNum(type, port, pos);
    if (tile_type) {
        if (useResourcePinNameFallback(type)) {
            // Packed logic pins are best resolved by site pin identity.
            u256 nodes = tile_type->pin_map.getNodesForPin(TILE_PIN_INPUT, modeledResourcePinName(tile_type, type, port, pos),
                                                            modeledSitePos(tile_type, pos),
                                                            cb_type ? cb_type->name : std::string{});
            if (endpointDebugEnabled()) {
                PNR_LOG1("FPGA", "endpoint input tile='{}' cb='{}' type='{}' port='{}' pos={} site_pos={} pin='{}' nodes={}",
                    makeName(), cb_type ? cb_type->name : std::string{}, type, port, pos,
                    modeledSitePos(tile_type, pos), modeledResourcePinName(tile_type, type, port, pos), nodes.str());
            }
            if (nodes != u256{}) {
                return nodes;
            }
        }
        int resource_node = modeledResourceNodeNum(tile_type, type, pos, local);
        u256 nodes = resource_node < 0 ? u256{} : tile_type->pin_map.getInputNodes(resource_node);
        if (nodes != u256{}) {
            return nodes;
        }
        nodes = tile_type->pin_map.getNodes(type, port, pos);
        if (nodes != u256{}) {
            return nodes;
        }
    }

    if (tile_type && useResourcePinNameFallback(type)) {
        if (local >= 0 && cb_type && (cb_type->local_input_nodes & (u256{0,1} << local)) != u256{}) {
            return u256{0,1} << local;
        }
        // Preserve abstract fallback only for tile types without loaded site endpoint models.
        if (local >= 0) {
            return u256{0,1} << local;
        }
        return u256{};
    }
    return local < 0 ? u256{} : (u256{0,1} << local);
}

u256 Tile::getOutputPinNodes(const std::string& type, const std::string& port, int pos) const
{
    int local = const_cast<Tile*>(this)->getNodeNum(type, port, pos);
    if (tile_type) {
        if (useResourcePinNameFallback(type)) {
            // Packed logic outputs are best resolved by site pin identity.
            u256 nodes = tile_type->pin_map.getNodesForPin(TILE_PIN_OUTPUT, modeledResourcePinName(tile_type, type, port, pos),
                                                            modeledSitePos(tile_type, pos),
                                                            cb_type ? cb_type->name : std::string{});
            if (endpointDebugEnabled()) {
                PNR_LOG1("FPGA", "endpoint output tile='{}' cb='{}' type='{}' port='{}' pos={} site_pos={} pin='{}' nodes={}",
                    makeName(), cb_type ? cb_type->name : std::string{}, type, port, pos,
                    modeledSitePos(tile_type, pos), modeledResourcePinName(tile_type, type, port, pos), nodes.str());
            }
            if (nodes != u256{}) {
                return nodes;
            }
        }
        int resource_node = modeledResourceNodeNum(tile_type, type, pos, local);
        u256 nodes = resource_node < 0 ? u256{} : tile_type->pin_map.getOutputNodes(resource_node);
        if (nodes != u256{}) {
            return nodes;
        }
    }

    return local < 0 ? u256{} : (u256{0,1} << local);
}

u256 Tile::getPinNodesForRouteType(const std::string& type, const std::string& port, int pos,
                                   TilePinNameType dir, const std::string& route_type) const
{
    // Endpoint route filtering lets adjacent route tiles own only their exact site-local nodes.
    if (!tile_type || route_type.empty()) {
        return dir == TILE_PIN_OUTPUT ? getOutputPinNodes(type, port, pos) : getPinNodes(type, port, pos);
    }

    if (useResourcePinNameFallback(type)) {
        bool strict_route_type = tile_type->elements.empty();
        u256 nodes = tile_type->pin_map.getNodesForPin(dir, modeledResourcePinName(tile_type, type, port, pos),
                                                       modeledSitePos(tile_type, pos), route_type, strict_route_type);
        if (endpointDebugEnabled()) {
            PNR_LOG1("FPGA", "endpoint route_type tile='{}' route='{}' dir={} type='{}' port='{}' pos={} nodes={}",
                makeName(), route_type, static_cast<int>(dir), type, port, pos, nodes.str());
        }
        return nodes;
    }

    int local = const_cast<Tile*>(this)->getNodeNum(type, port, pos);
    int resource_node = modeledResourceNodeNum(tile_type, type, pos, local);
    if (resource_node >= 0) {
        u256 nodes = dir == TILE_PIN_OUTPUT
            ? tile_type->pin_map.getOutputNodes(resource_node)
            : tile_type->pin_map.getInputNodes(resource_node);
        if (nodes != u256{}) {
            return nodes;
        }
    }
    return u256{};
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
    return (pin_state.leased_nodes & (u256{0,1} << local)) != u256{};
}

int Tile::getNodeNum(std::string type, std::string port, int pos)
{
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

    if (type == "IBUF" && port == "O" && cb_type) {
        int node = cb_type->localNodeNum("LOGIC_OUTS18");
        if (node >= 0) {
            cb_type->rememberNodeName(CB_NODE_LOCAL, node, "LOGIC_OUTS18");
            return node;
        }
        else {
            node = cb_type->localNodeNum("LOGIC_OUTS_L18");
        }
        if (node >= 0) {
            cb_type->rememberNodeName(CB_NODE_LOCAL, node, "LOGIC_OUTS_L18");
            return node;
        }
    }
    if (type == "OBUF" && port == "I" && cb_type) {
        for (const char* name : {"IMUX34", "IMUX_L34", "IMUX_R34"}) {
            int node = cb_type->localNodeNum(name);
            if (node >= 0) {
                cb_type->rememberNodeName(CB_NODE_LOCAL, node, name);
                return node;
            }
        }
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
        if (port == "O") return indexedNode(mux_out, muxDataBelFromPlacedPos(type, "I0", pos));
        if (port == "S") return indexedNode(ff_d, muxControlBelFromPlacedPos(type, pos));
    }
    return -1;
}

void Tile::assign(rtl::Inst* inst)
{
    PNR_ASSERT(inst->tile.peer == nullptr, "assigning tile {} to already assigned inst {}", makeName(), inst->makeName(), inst->tile->makeName());
    inst->tile.set(static_cast<Referable<Tile>*>(this));
}

int Tile::tryAdd(rtl::Inst* inst)  // it's not SRL
{
    PNR_ASSERT(coord.x > -1 && coord.y > -1, "trying to add inst '{}' to a tile '{}' with coords -1", inst->makeName(), makeName());
    if (!inst->cell_ref.peer) {
        return -1;
    }
    ElementType type = instElementType(*inst);
    int pos = -1;
    if (!tryElementPlacement(*this, inst, type, pos)) {
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
    markVoidNetsForTile(*this);
    if (packDebugEnabled()) {
        std::fprintf(stderr, "pack-debug commit inst=%s element=%s tile=%s pos=%d bit=%d\n",
            inst->makeName().c_str(), elementTypeName(type), makeName().c_str(), pos, bit);
        printTypeMasks("pack-debug   free-after", elements_free);
    }
    return pos;
}
