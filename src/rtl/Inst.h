#pragma once

#include "Module.h"
#include "Inst.h"
#include "Conn.h"
#include "Cell.h"
#include "referable.h"
#include "TimingPath.h"
#include "RegBunch.h"

#include <vector>
#include <list>

inline std::string shortenName(const std::string& in, size_t limit)
{
    if (in.length() <= limit) {
        return in;
    }
    return in.substr(0,limit/3) + "..." + in.substr(in.length()-limit/3,limit/3);
}

namespace clk {
struct TimingPath;
}
namespace pnr {
struct RegBunch;
}

namespace rtl
{

struct Inst
{
    // must have
    int depth = -1;
    int height = -1;
    Ref<Cell> cell_ref;
    int cnt_inputs = -1;
    int cnt_outputs = -1;

    // optional
    Ref<Inst> parent_ref;  // can be zero for top
    std::vector<Referable<Conn>> conns;
    std::list<Referable<Inst>> insts;
    Ref<TimingPath> timing;  // self-clearing pointer to timing info starting from this Inst
    Ref<RegBunch> placing;  // self-clearing pointer to timing info starting from this Inst
    int mark = 0;  // for traversal marks - to visit one time
    int used_in_bunches = 0;
    int cnt_clocks = 0;  // clk inputs
    bool locked = false;  // for traversal locks - cycle prevention

    std::string makeName(size_t limit = 200);
    Conn* operator [](const std::string& port_name);
    static int genMark();
};

}
