#include "Tile.h"

using namespace fpga;

int Tile::getNodeNum(std::string type, std::string port, int pos)
{
    if (type.find("FD") == 0) {
        if (port == "C") return 0 + 64*pos;
        if (port == "CE") return 1 + 64*pos;
        if (port == "D") return 2 + 64*pos;
        if (port == "Q") return 3 + 64*pos;
        if (port == "R") return 4 + 64*pos;
        if (port == "S") return 5 + 64*pos;
        if (port == "CLR") return 6 + 64*pos;
        if (port == "PRE") return 7 + 64*pos;
        if (port == "EN") return 8 + 64*pos;
        if (port == "SRST") return 9 + 64*pos;
        if (port == "ARST") return 10 + 64*pos;
    }
    if (type.find("LUT") == 0) {
        if (port == "I0") return 16 + 64*pos;
        if (port == "AI") return 17 + 64*pos;
        if (port == "I1") return 18 + 64*pos;
        if (port == "I2") return 19 + 64*pos;
        if (port == "I3") return 20 + 64*pos;
        if (port == "I4") return 21 + 64*pos;
        if (port == "I5") return 22 + 64*pos;
        if (port == "A0") return 23 + 64*pos;
        if (port == "A1") return 24 + 64*pos;
        if (port == "A2") return 25 + 64*pos;
        if (port == "A3") return 26 + 64*pos;
        if (port == "A4") return 27 + 64*pos;
        if (port == "A5") return 28 + 64*pos;
        if (port == "MC31") return 29 + 64*pos;
        if (port == "DI1") return 30 + 64*pos;
        if (port == "DI2") return 31 + 64*pos;
        if (port == "WCLK") return 32 + 64*pos;
        if (port == "DPRA0") return 33 + 64*pos;
        if (port == "DPRA1") return 34 + 64*pos;
        if (port == "DPRA2") return 35 + 64*pos;
        if (port == "DPRA3") return 36 + 64*pos;
        if (port == "DPRA4") return 37 + 64*pos;
        if (port == "DPRA5") return 38 + 64*pos;
        if (port == "O6") return 39 + 64*pos;
        if (port == "O5") return 40 + 64*pos;
        if (port == "WE") return 41 + 64*pos;
    }
    if (type.find("CARRY") == 0) {
        if (port == "CI") return 42 + 64*pos;
        if (port == "CYINIT") return 43 + 64*pos;
        if (port == "DI0") return 44 + 64*pos;
        if (port == "DI1") return 45 + 64*pos;
        if (port == "DI2") return 46 + 64*pos;
        if (port == "DI3") return 47 + 64*pos;
        if (port == "S0") return 48 + 64*pos;
        if (port == "S1") return 49 + 64*pos;
        if (port == "S2") return 50 + 64*pos;
        if (port == "S3") return 51 + 64*pos;
        if (port == "C0") return 52 + 64*pos;
        if (port == "C1") return 53 + 64*pos;
        if (port == "C2") return 54 + 64*pos;
        if (port == "C3") return 55 + 64*pos;
        if (port == "O0") return 56 + 64*pos;
        if (port == "O1") return 57 + 64*pos;
        if (port == "O2") return 58 + 64*pos;
        if (port == "O3") return 59 + 64*pos;
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
            ++regs_cnt;

            assign(inst);
            return pos;
        }
    }
    if (inst->cell_ref->type.find("LUT") == 0) {
        int pos = (luts6cnt + luts5cnt/2 + luts1cnt/4)*4 + 3;
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
        carry = 4;
        assign(inst);
        return pos;
    }
    if (inst->cell_ref->type.find("MUX") == 0 && mux == 0 && luts6cnt <= 2) {
        int pos = mux*4 + 1;
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
