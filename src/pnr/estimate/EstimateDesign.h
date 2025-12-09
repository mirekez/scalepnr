#pragma once

#include "Design.h"
#include "RegBunch.h"
#include "Inst.h"
#include "Clocks.h"

#include <vector>
#include <string>

namespace technology
{
    struct Tech;
}

namespace pnr
{

struct EstimateDesign
{
    std::list<Referable<RegBunch>> data_outs;
    long travers_mark = -1;
    technology::Tech* tech = nullptr;
    clk::Clocks* clocks = nullptr;

    void findTopOutputs(rtl::Design& rtl);

    void estimateDesign(rtl::Design& rtl);
    bool recurseReg(Referable<RegBunch>* bunch, rtl::Inst* reg, int depth = 0, int depth_comb = 0);
    void recurseComb(Referable<RegBunch>* bunch, rtl::Inst* comb, rtl::Conn* from, int depth = 0, int depth_comb = 0, double bottom_delay = 0, bool grab = false);
    int aggregateRegs(Referable<RegBunch>* bunch, int depth = 0, int count_empty = 0);
    void sortBunches(std::list<Referable<RegBunch>>* bunch_list, int depth = 0);
    void printBunches(std::list<Referable<RegBunch>>* bunch_list = nullptr, int depth = 0);
};


}
