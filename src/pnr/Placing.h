#pragma once

#include "Design.h"
#include "Bunch.h"
#include "Inst.h"
#include "on_return.h"

#include <vector>
#include <string>

namespace pnr
{

struct Placing
{
    struct DataOut
    {
        rtl::Inst* reg_in;
        Bunch bunch;
    };

    std::vector<DataOut> data_outs;
    int mark = rtl::Inst::genMark();
    const std::multimap<std::string,std::string>* clocked_ports = nullptr;
    const std::multimap<std::string,std::string>* iobufs_ports = nullptr;

    void findTopOutputs(rtl::Design& rtl, const std::multimap<std::string,std::string>& iobufs_ports);

    void recurseComb(Bunch* bunch, rtl::Inst* comb, int depth_regs = 0, int depth_comb = 0);
    void recurseReg(Bunch* bunch, int depth_regs = 0, int depth_comb = 0);
};


}
