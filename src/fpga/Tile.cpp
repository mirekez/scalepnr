#include "Tile.h"
#include "Net.h"

#include <cstdlib>
#include <cstddef>
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
    return (pos % 128) / 4;
}

int siteIndexFromPlacedPos(int pos)
{
    return pos >= 0 ? pos / 128 : 0;
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

bool isFd(const rtl::Inst& inst)
{
    // Classify flip-flop primitives by generic cell type prefix.
    return inst.cell_ref.peer && inst.cell_ref.peer->type.find("FD") == 0;
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
int siteCapacity(const Tile& tile);

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

bool directlyChained(rtl::Inst& a, rtl::Inst& b)
{
    // Shared output-local resources may coexist only when connected as a direct chain.
    return drivesInput(a, b) || drivesInput(b, a);
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
            net->void_net = true;
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
            if (isMux(*sink) && drivesInput(*driver, *sink)) {
                markVoidNetsBetween(*driver, *sink);
            }
        }
    }
}

rtl::Inst* instAt(Tile& tile, bool (*match)(const rtl::Inst&), int pos)
{
    // Find a placed instance of a requested class at an exact tile position.
    for (rtl::Inst* inst : assignedInsts(tile)) {
        if (inst && match(*inst) && inst->pos == pos) {
            return inst;
        }
    }
    return nullptr;
}

rtl::Inst* inputDriver(rtl::Inst& inst, const std::string& port_name)
{
    // Follow one input pin to the primitive that drives this local shape.
    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN || conn.port_ref->name != port_name) {
            continue;
        }
        rtl::Conn* driver_conn = conn.follow();
        return driver_conn && driver_conn->inst_ref.peer ? driver_conn->inst_ref.peer : nullptr;
    }
    return nullptr;
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

u256 outputLocalMask(Tile& tile, rtl::Inst* inst, int pos)
{
    // Resolve the resource output locals for exclusive-driver placement checks.
    if (!inst || !inst->cell_ref.peer || !isMux(*inst)) {
        return {};
    }
    return tile.getOutputPinNodes(inst->cell_ref->type, "O", pos);
}

bool outputLocalAvailable(Tile& tile, rtl::Inst* inst, int pos,
                          const std::vector<std::pair<rtl::Inst*, int>>& reservations)
{
    // Reject unrelated resources that would drive the same tile-local output.
    u256 candidate = outputLocalMask(tile, inst, pos);
    if (candidate == u256{}) {
        return true;
    }
    for (rtl::Inst* existing : assignedInsts(tile)) {
        if (!existing || existing == inst || !existing->cell_ref.peer) {
            continue;
        }
        u256 occupied = outputLocalMask(tile, existing, existing->pos);
        if ((candidate & occupied) != u256{} && !directlyChained(*inst, *existing)) {
            return false;
        }
    }
    for (const auto& [reserved_inst, reserved_pos] : reservations) {
        if (!reserved_inst || reserved_inst == inst || !reserved_inst->cell_ref.peer) {
            continue;
        }
        u256 reserved = outputLocalMask(tile, reserved_inst, reserved_pos);
        if ((candidate & reserved) != u256{} && !directlyChained(*inst, *reserved_inst)) {
            return false;
        }
    }
    return true;
}

bool positionAvailable(Tile& tile, rtl::Inst* inst, int pos, bool (*match)(const rtl::Inst&),
                       const std::vector<std::pair<rtl::Inst*, int>>& reservations)
{
    // Validate one pending packed-shape position against existing and reserved users.
    if (!inst || !match(*inst) || !canHost(tile, inst, pos) || !outputLocalAvailable(tile, inst, pos, reservations)) {
        return false;
    }
    if (inst->tile.peer) {
        return inst->tile.peer == static_cast<Referable<Tile>*>(&tile) && inst->pos == pos;
    }
    for (const auto& [reserved_inst, reserved_pos] : reservations) {
        if (reserved_inst == inst) {
            return reserved_pos == pos;
        }
        if (reserved_pos == pos && match(*reserved_inst)) {
            return false;
        }
    }
    return !instAt(tile, match, pos);
}

bool reservePosition(Tile& tile, rtl::Inst* inst, int pos, bool (*match)(const rtl::Inst&),
                     std::vector<std::pair<rtl::Inst*, int>>& reservations)
{
    // Add one instance to the pending packed-shape placement set.
    if (!positionAvailable(tile, inst, pos, match, reservations)) {
        return false;
    }
    if (!inst->tile.peer) {
        reservations.emplace_back(inst, pos);
    }
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
        return (tile.tile_type->name.rfind("CLBLL_", 0) == 0 || tile.tile_type->name.rfind("CLBLM_", 0) == 0)
            && carryLutSlotCompatible(tile, inst, pos);
    }
    return true;
}

bool collectLeafMuxShape(Tile& tile, rtl::Inst* mux, int site, int pair_start,
                         std::vector<std::pair<rtl::Inst*, int>>& reservations)
{
    // Collect a mux with two direct LUT drivers into adjacent BEL slots.
    rtl::Inst* i0 = inputDriver(*mux, "I0");
    rtl::Inst* i1 = inputDriver(*mux, "I1");
    if (!i0 || !i1 || !isLut(*i0) || !isLut(*i1)) {
        return false;
    }
    return reservePosition(tile, i0, site*128 + pair_start*4 + 3, isLut, reservations)
        && reservePosition(tile, i1, site*128 + (pair_start + 1)*4 + 3, isLut, reservations)
        && reservePosition(tile, mux, site*128 + pair_start*4 + 1, isMux, reservations);
}

bool collectMuxTreeShape(Tile& tile, rtl::Inst* mux, int site,
                         std::vector<std::pair<rtl::Inst*, int>>& reservations)
{
    // Collect a second-level mux with two child mux/LUT local shapes.
    rtl::Inst* i0 = inputDriver(*mux, "I0");
    rtl::Inst* i1 = inputDriver(*mux, "I1");
    if (!i0 || !i1 || !isMux(*i0) || !isMux(*i1)) {
        return false;
    }
    return collectLeafMuxShape(tile, i0, site, 0, reservations)
        && collectLeafMuxShape(tile, i1, site, 2, reservations)
        && reservePosition(tile, mux, site*128 + 1, isMux, reservations);
}

void placeReservedShape(Tile& tile, const std::vector<std::pair<rtl::Inst*, int>>& reservations)
{
    // Commit all unplaced instances collected for one local shape.
    for (auto& [inst, pos] : reservations) {
        if (inst->tile.peer) {
            continue;
        }
        tile.assign(inst);
        inst->coord = tile.coord;
        inst->pos = pos;
        inst->outline.x = tile.coord.x + 0.25f*(pos%4);
        inst->outline.y = tile.coord.y + 0.25f*(pos/4);
    }
    markVoidNetsForTile(tile);
}

int tryAddMuxShape(Tile& tile, rtl::Inst* inst)
{
    // Prefer local mux/LUT tree placement when the netlist exposes direct drivers.
    for (int site = 0; site < siteCapacity(tile); ++site) {
        std::vector<std::pair<rtl::Inst*, int>> reservations;
        if (collectMuxTreeShape(tile, inst, site, reservations)) {
            placeReservedShape(tile, reservations);
            tile.luts6cnt += 4;
            tile.mux += 3;
            return site*128 + 1;
        }
        reservations.clear();
        if (collectLeafMuxShape(tile, inst, site, 0, reservations)) {
            placeReservedShape(tile, reservations);
            tile.luts6cnt += 2;
            tile.mux += 1;
            return site*128 + 1;
        }
        reservations.clear();
        if (collectLeafMuxShape(tile, inst, site, 2, reservations)) {
            placeReservedShape(tile, reservations);
            tile.luts6cnt += 2;
            tile.mux += 1;
            return site*128 + 9;
        }
    }
    return -1;
}

int siteCapacity(const Tile& tile)
{
    // Use loaded site models to decide how many independent logic sites exist.
    return tile.tile_type && !tile.tile_type->sites.empty()
        ? static_cast<int>(tile.tile_type->sites.size())
        : 1;
}

bool useResourcePinNameFallback(const std::string& type)
{
    // Restrict resource-pin-name fallback to logic primitives with packed site pins.
    return type.find("LUT") == 0
        || type.find("FD") == 0
        || type.find("CARRY") == 0
        || type.find("MUX") == 0;
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

    static constexpr char bel_prefix[4] = {'A', 'B', 'C', 'D'};
    int bel = belIndexFromPlacedPos(pos);
    char prefix = bel >= 0 && bel < 4 ? bel_prefix[bel] : 'A';

    if (type.find("LUT") == 0) {
        if (port == "I0" || port == "A1") return std::string{prefix} + "1";
        if (port == "I1" || port == "A2") return std::string{prefix} + "2";
        if (port == "I2" || port == "A3") return std::string{prefix} + "3";
        if (port == "I3" || port == "A4") return std::string{prefix} + "4";
        if (port == "I4" || port == "A5") return std::string{prefix} + "5";
        if (port == "I5" || port == "A6") return std::string{prefix} + "6";
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

bool endpointDebugEnabled()
{
    // Optional endpoint trace for diagnosing database-to-local pin resolution.
    return std::getenv("SCALEPNR_ENDPOINT_DEBUG") != nullptr;
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
        u256 nodes = local < 0 ? u256{} : tile_type->pin_map.getInputNodes(local);
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
        u256 nodes = local < 0 ? u256{} : tile_type->pin_map.getOutputNodes(local);
        if (nodes != u256{}) {
            return nodes;
        }
    }

    return local < 0 ? u256{} : (u256{0,1} << local);
}

int Tile::getResourceNodeNum(const std::string& type, const std::string& port, int pos, TilePinNameType dir, int local) const
{
    // Resolve endpoint identity after routing selects the concrete local node.
    int preferred = const_cast<Tile*>(this)->getNodeNum(type, port, pos);
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
        int bel = belIndexFromPlacedPos(pos);
        if (port == "I0" || port == "I1" || port == "O") return indexedNode(mux_out, bel);
        if (port == "S") return indexedNode(ff_d, bel);
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
    if (inst->cell_ref->type.find("FD") == 0) {
        for (int site = 0; site < siteCapacity(*this); ++site) {
            for (int bel = 0; bel < 4; ++bel) {
                int pos = site*128 + bel*4 + 0;
                if (instAt(*this, isFd, pos)) {
                    continue;
                }
                if (!canHost(*this, inst, pos)) {
                    continue;
                }
                ++regs_cnt;

                assign(inst);
                markVoidNetsForTile(*this);
                return pos;
            }
        }
    }
    if (inst->cell_ref->type.find("LUT") == 0) {
        for (int site = 0; site < siteCapacity(*this); ++site) {
            for (int bel = 0; bel < 4; ++bel) {
                int pos = site*128 + bel*4 + 3;
                if (lutAtBel(*this, site, bel)) {
                    continue;
                }
                if (!canHost(*this, inst, pos)) {
                    continue;
                }
                if (inst->cnt_inputs == 1) {
                    ++luts1cnt;
                }
                else if (inst->cnt_inputs == 6) {
                    ++luts6cnt;
                }
                else {
                    ++luts5cnt;
                }
                assign(inst);
                markVoidNetsForTile(*this);
                return pos;
            }
        }
    }
    if (inst->cell_ref->type.find("CARRY") == 0) {
        for (int site = 0; site < siteCapacity(*this); ++site) {
            if (carryInSite(*this, site)) {
                continue;
            }
            int pos = site*128 + 2;
            if (!canHost(*this, inst, pos)) {
                continue;
            }
            carry += 4;
            assign(inst);
            markVoidNetsForTile(*this);
            return pos;
        }
    }
    if (inst->cell_ref->type.find("MUX") == 0) {
        if (int pos = tryAddMuxShape(*this, inst); pos >= 0) {
            return pos;
        }
    }
    if (inst->cell_ref->type.find("MUX") == 0 && mux == 0 && luts6cnt <= 2) {
        for (int site = 0; site < siteCapacity(*this); ++site) {
            int pos = site*128 + 1;
            if (instAt(*this, isMux, pos)) {
                continue;
            }
            if (!canHost(*this, inst, pos) || !outputLocalAvailable(*this, inst, pos, {})) {
                continue;
            }
            luts6cnt += 2;
            mux = 1;
            assign(inst);
            markVoidNetsForTile(*this);
/*        for (auto& conn : inst->conns) {
            rtl::Conn* curr = &conn;
            if (curr->port_ref->type == rtl::Port::PORT_IN) {
                curr = curr->follow();
                PNR_ASSERT (curr && (curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global), "wrong connections to MUX");
                assign(curr->inst_ref.peer);
            }
        }*/
//        PNR_ASSERT(cnt == 2, "MUX {} has {} inputs", inst->makeName(), cnt);
            return pos;
        }
    }
    return -1;
}
