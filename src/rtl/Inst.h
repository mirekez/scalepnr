#pragma once

#include "Module.h"
#include "Inst.h"
#include "Conn.h"
#include "Cell.h"
#include "referable.h"
//#include "BelType.h"

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
namespace fpga {
struct Tile;
}

namespace rtl
{

struct CombStats
{
    int top_max_length = 0;
    int top_max_comb = 0;
    double top_max_delay = 0;
    int bottom_max_length = 0;
    int bottom_max_comb = 0;
    double bottom_max_delay = 0;

    double max_deficit = -100;
};

struct OutlineInfo
{
    float x;
    float y;
    bool fixed = false;
};


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

    CombStats stats;
    OutlineInfo outline;

    Ref<clk::TimingPath> timing;  // self-clearing pointer to timing info
    Ref<pnr::RegBunch> bunch_ref;  // self-clearing pointer to placing info
    Ref<fpga::Tile> tile;  // self-clearing pointer to tile info
    long mark = 0;  // for traversal marks - to visit one time
//    int used_in_bunches = 0;
    int cnt_clocks = 0;  // clk inputs
    bool locked = false;  // for traversal locks - cycle prevention

//    fpga::BelType type;

    std::string makeName(size_t limit = 200);
    Conn* operator [](const std::string& port_name);
    static long mark_counter;
    static long genMark()
    {
        return ++mark_counter;
    }
};

}
