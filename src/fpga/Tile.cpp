#include "Tile.h"

using namespace fpga;

void Tile::assign(rtl::Inst* inst)
{
    PNR_ASSERT(inst->tile.peer == nullptr, "assigning tile {} to already assigned inst {}", makeName(), inst->makeName());
    inst->tile.set(static_cast<Referable<Tile>*>(this));
}

int Tile::tryAdd(rtl::Inst* inst)  // it's not SRL
{
    if (inst->cell_ref->type == "FDRE") {
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
    if (inst->cell_ref->type == "CARRY4" && carry == 0) {
        int pos = carry*4 + 2;
        carry = 4;
        assign(inst);
        return pos;
    }
    if (inst->cell_ref->type == "MUXF7" && mux7 == 0 && luts6cnt <= 2) {
        int pos = mux7*4 + 1;
        luts6cnt += 2;
        mux7 = 1;
        assign(inst);
        int cnt = 0;
        for (auto& conn : inst->conns) {
            rtl::Conn* curr = &conn;
            if (curr->port_ref->type == rtl::Port::PORT_IN) {
                curr = curr->follow();
                PNR_ASSERT (curr && (curr->inst_ref->cell_ref->module_ref->is_blackbox || curr->port_ref->is_global), "wrong connections to MUXF7");
                assign(curr->inst_ref.peer);
            }
            ++cnt;
        }
        PNR_ASSERT(cnt == 2, "MUXF7 {} has {} inputs", inst->makeName(), cnt);
        return pos;
    }
    return 0;
}
