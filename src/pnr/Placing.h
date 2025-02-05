#pragma once

#include "Design.h"
#include "RegBunch.h"
#include "TileSet.h"
#include "Inst.h"
#include "Tech.h"
#include "Clocks.h"

#include <vector>
#include <string>

namespace pnr
{

struct Placing
{
    struct DataOut
    {
        rtl::Inst* reg;
        Referable<RegBunch> bunch;
    };

    std::vector<DataOut> data_outs;
    int travers_mark = -1;
    tech::Tech* tech = nullptr;
    clk::Clocks* clocks = nullptr;

    void findTopOutputs(rtl::Design& rtl);

    void recurseComb(Referable<RegBunch>* bunch, rtl::Inst* comb, rtl::Conn* from, int depth = 0, int depth_comb = 0, double bottom_delay = 0, bool capture = false);
    void recurseReg(Referable<RegBunch>* bunch, rtl::Inst* reg, int depth = 0, int depth_comb = 0);

    void calculateDesign(rtl::Design& rtl)
    {
        findTopOutputs(rtl);
        travers_mark = rtl::Inst::genMark();
        for (auto& data_out : data_outs) {
            data_out.bunch.reg = data_out.reg;
            recurseReg(&data_out.bunch, data_out.bunch.reg);
        }
    }

//    void packBunch(Inst* reg, TileSet& tiles, int depth = 0);
};


}
