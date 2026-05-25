#include "Tile.h"

#include <cstddef>

using namespace fpga;

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
    if (pos >= 128) {
        pos -= 128;
    }
    return (pos % 128) / 4;
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
    auto* tile_ref = Ref<Tile>::fromBase(ref);
    return reinterpret_cast<rtl::Inst*>(reinterpret_cast<char*>(tile_ref) - offsetof(rtl::Inst, tile));
}

std::vector<rtl::Inst*> assignedInsts(Tile& tile)
{
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
    return inst.cell_ref.peer && inst.cell_ref.peer->type.find("FD") == 0;
}

bool isLut(const rtl::Inst& inst)
{
    return inst.cell_ref.peer && inst.cell_ref.peer->type.find("LUT") == 0;
}

bool isCarry(const rtl::Inst& inst)
{
    return inst.cell_ref.peer && inst.cell_ref.peer->type.find("CARRY") == 0;
}

bool isMux(const rtl::Inst& inst)
{
    return inst.cell_ref.peer && inst.cell_ref.peer->type.find("MUX") == 0;
}

bool portBitMatches(const rtl::Port& port, const std::string& name, int bit)
{
    std::string port_name = port.name;
    int port_bit = extractIndexedPort(port_name);
    if (port_bit < 0) {
        port_bit = port.bitnum;
    }
    return port_name == name && port_bit == bit;
}

rtl::Conn* carryInputDriver(rtl::Inst& inst, const std::string& name, int bit)
{
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

rtl::Inst* instAt(Tile& tile, bool (*match)(const rtl::Inst&), int pos)
{
    for (rtl::Inst* inst : assignedInsts(tile)) {
        if (inst && match(*inst) && inst->pos == pos) {
            return inst;
        }
    }
    return nullptr;
}

rtl::Inst* lutAtBel(Tile& tile, int bel)
{
    for (rtl::Inst* inst : assignedInsts(tile)) {
        if (inst && isLut(*inst) && belIndexFromPlacedPos(inst->pos) == bel) {
            return inst;
        }
    }
    return nullptr;
}

rtl::Inst* carryInTile(Tile& tile)
{
    for (rtl::Inst* inst : assignedInsts(tile)) {
        if (inst && isCarry(*inst)) {
            return inst;
        }
    }
    return nullptr;
}

bool carryLutSlotCompatible(Tile& tile, rtl::Inst* inst, int pos)
{
    int bel = belIndexFromPlacedPos(pos);

    if (isLut(*inst)) {
        rtl::Inst* carry = carryInTile(tile);
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
            rtl::Inst* lut = lutAtBel(tile, bel);
            if (lut && !hasOutputConn(*lut, s_driver)) {
                return false;
            }
        }
    }

    return true;
}

bool canHost(Tile& tile, rtl::Inst* inst, int pos)
{
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

bool useResourcePinNameFallback(const std::string& type)
{
    return type.find("LUT") == 0
        || type.find("FD") == 0
        || type.find("CARRY") == 0
        || type.find("MUX") == 0;
}

std::string normalizedResourcePinName(std::string type, std::string port, int pos)
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
        if (port == "I0") return "F7AMUX";
        if (port == "I1") return "F7BMUX";
        if (port == "S") return "F7AMUX";
        if (port == "O") return "F7AMUX";
    }
    return port;
}

}

u256 Tile::getPinNodes(const std::string& type, const std::string& port, int pos) const
{
    if (type == "OBUF" && port == "I" && cb_type) {
        u256 nodes;
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
        u256 nodes = local < 0 ? u256{} : tile_type->pin_map.getInputNodes(local);
        if (nodes != u256{}) {
            return nodes;
        }
        if (useResourcePinNameFallback(type)) {
            nodes = tile_type->pin_map.getNodesForPin(TILE_PIN_INPUT, normalizedResourcePinName(type, port, pos));
            if (nodes != u256{}) {
                return nodes;
            }
        }
        nodes = tile_type->pin_map.getNodes(type, port, pos);
        if (nodes != u256{}) {
            return nodes;
        }
    }

    return local < 0 ? u256{} : (u256{0,1} << local);
}

u256 Tile::getOutputPinNodes(const std::string& type, const std::string& port, int pos) const
{
    int local = const_cast<Tile*>(this)->getNodeNum(type, port, pos);
    if (tile_type) {
        u256 nodes = local < 0 ? u256{} : tile_type->pin_map.getOutputNodes(local);
        if (nodes != u256{}) {
            return nodes;
        }
    }

    return local < 0 ? u256{} : (u256{0,1} << local);
}

int Tile::getResourceNodeNum(const std::string& type, const std::string& port, int pos, TilePinNameType dir, int local) const
{
    int preferred = const_cast<Tile*>(this)->getNodeNum(type, port, pos);
    if (!tile_type) {
        return preferred;
    }
    return tile_type->pin_map.findResourceNode(dir, useResourcePinNameFallback(type) ? normalizedResourcePinName(type, port, pos) : std::string{}, local, preferred);
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
        if (port == "I0") return 60 + 64*pos;
        if (port == "I1") return 61 + 64*pos;
        if (port == "S") return 62 + 64*pos;
        if (port == "O") return 63 + 64*pos;
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
        for (int bel = 0; bel < 4; ++bel) {
            int pos = bel*4 + 0;
            if (instAt(*this, isFd, pos)) {
                continue;
            }
            if (!canHost(*this, inst, pos)) {
                continue;
            }
            ++regs_cnt;

            assign(inst);
            return pos;
        }
    }
    if (inst->cell_ref->type.find("LUT") == 0) {
        for (int bel = 0; bel < 4; ++bel) {
            int pos = bel*4 + 3;
            if (lutAtBel(*this, bel)) {
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
            return pos;
        }
    }
    if (inst->cell_ref->type.find("CARRY") == 0 && !carryInTile(*this)) {
        int pos = 2;
        if (!canHost(*this, inst, pos)) {
            return -1;
        }
        carry = 4;
        assign(inst);
        return pos;
    }
    if (inst->cell_ref->type.find("MUX") == 0 && mux == 0 && luts6cnt <= 2) {
        int pos = 1;
        if (instAt(*this, isMux, pos)) {
            return -1;
        }
        if (!canHost(*this, inst, pos)) {
            return -1;
        }
        luts6cnt += 2;
        mux = 1;
        assign(inst);
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
    return -1;
}
