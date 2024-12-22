#pragma once

#include "Design.h"
#include "RegBunch.h"
#include "Inst.h"
#include "Tech.h"

#include <vector>
#include <string>

namespace pnr
{

struct Placing
{
    struct DataOut
    {
        rtl::Inst* reg_in;
        Referable<rtl::RegBunch> bunch;
    };

    std::vector<DataOut> data_outs;
    int mark = -1;
    tech::Tech* tech = nullptr;

    void findTopOutputs(rtl::Design& rtl);

    void recurseComb(Referable<rtl::RegBunch>* bunch, rtl::Inst* comb, rtl::CombStats* stats, bool clear, rtl::Conn* from, int depth_regs = 0, int depth_comb = 0);
    void recurseReg(Referable<rtl::RegBunch>* bunch, bool clear, int depth_regs = 0, int depth_comb = 0);

    void calculateDesign(rtl::Design& rtl)
    {
        findTopOutputs(rtl);
        for (auto& data_out : data_outs) {
            data_out.bunch.reg_in = data_out.reg_in;
            recurseReg(&data_out.bunch, false);  // first time to accumulate
            recurseReg(&data_out.bunch, true);  // second time to take and clear, must touch same insts in same order
        }
    }
};


}
