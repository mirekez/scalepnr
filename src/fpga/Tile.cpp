#include "Tile.h"

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

bool canHost(Tile& tile, rtl::Inst* inst, int pos)
{
    (void)pos;
    if (!tile.tile_type) {
        return false;
    }

    const std::string& type = inst->cell_ref->type;
    if (type.find("FD") == 0 || type.find("LUT") == 0 || type.find("CARRY") == 0 || type.find("MUX") == 0) {
        return tile.tile_type->name.rfind("CLBLL_", 0) == 0 || tile.tile_type->name.rfind("CLBLM_", 0) == 0;
    }
    return true;
}

}

u256 Tile::getPinNodes(const std::string& type, const std::string& port, int pos) const
{
    int local = const_cast<Tile*>(this)->getNodeNum(type, port, pos);
    if (tile_type) {
        u256 nodes = local < 0 ? u256{} : tile_type->pin_map.getInputNodes(local);
        if (nodes != u256{}) {
            return nodes;
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

    static constexpr int lut_out[4] = {16, 80, 144, 212};
    static constexpr int lut_in0[4] = {17, 81, 145, 213};
    static constexpr int lut_in1[4] = {18, 82, 146, 214};
    static constexpr int lut_in2[4] = {19, 83, 147, 215};
    static constexpr int lut_in3[4] = {20, 84, 148, 216};
    static constexpr int lut_in4[4] = {21, 85, 149, 217};
    static constexpr int lut_in5[4] = {22, 86, 150, 218};
    static constexpr int ff_d[4] = {31, 95, 130, 198};
    static constexpr int ff_q[4] = {1, 65, 129, 197};

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
        if (port == "DI0" || port == "DI1" || port == "DI2" || port == "DI3") return indexedNode(lut_out, bel);
        if (port == "S0" || port == "S1" || port == "S2" || port == "S3") return indexedNode(lut_in0, bel);
        if (port == "C0" || port == "C1" || port == "C2") return indexedNode(ff_d, bel);
        if (port == "C3") return 63;
        if (port == "O0" || port == "O1" || port == "O2" || port == "O3") return indexedNode(ff_d, bel);
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
        if (regs_cnt < 4) {
            int pos = regs_cnt*4 + 0;
            if (!canHost(*this, inst, pos)) {
                return -1;
            }
            ++regs_cnt;

            assign(inst);
            return pos;
        }
    }
    if (inst->cell_ref->type.find("LUT") == 0) {
        int pos = (luts6cnt + luts5cnt/2 + luts1cnt/4)*4 + 3;
        if (!canHost(*this, inst, pos)) {
            return -1;
        }
        if (inst->cnt_inputs == 1 && luts1cnt < 4 && luts6cnt < 4) {
            ++luts1cnt;
            assign(inst);
            return pos;
        }
        else
        if (inst->cnt_inputs == 6 && luts6cnt + luts1cnt < 4) {
            ++luts6cnt;
            assign(inst);
            return pos;
        }
        else
        if (luts5cnt < 4) {
            ++luts5cnt;
            assign(inst);
            return pos;
        }
    }
    if (inst->cell_ref->type.find("CARRY") == 0 && carry == 0) {
        int pos = carry*4 + 2;
        if (!canHost(*this, inst, pos)) {
            return -1;
        }
        carry = 4;
        assign(inst);
        return pos;
    }
    if (inst->cell_ref->type.find("MUX") == 0 && mux == 0 && luts6cnt <= 2) {
        int pos = mux*4 + 1;
        if (!canHost(*this, inst, pos)) {
            return -1;
        }
        luts6cnt += 2;
        mux = 1;
        assign(inst);
        int cnt = 0;
/*        for (auto& conn : inst->conns) {
            rtl::Conn* curr = &conn;
            if (curr->port_ref->type == rtl::Port::PORT_IN) {
                curr = curr->follow();
                PNR_ASSERT (curr && (curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global), "wrong connections to MUX");
                assign(curr->inst_ref.peer);
            }
            ++cnt;
        }*/
//        PNR_ASSERT(cnt == 2, "MUX {} has {} inputs", inst->makeName(), cnt);
        return pos;
    }
    return -1;
}
