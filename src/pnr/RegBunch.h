#pragma once

#include "Inst.h"
#include "referable.h"
#include "Clock.h"
#include "TileSet.h"

namespace pnr
{

struct BunchLink
{
    // must have
    double delay;
    double deficit;
    rtl::Conn* conn;
    int length;
};

struct RegBunch
{
    // must have
    rtl::Inst* reg = nullptr;
    // optional
    Ref<rtl::Clock> clk;  // we can use simple pointet here if not afraid of someone deletes clock
    std::list<Referable<RegBunch>> sub_bunches;  // we use list to sort easily
    std::vector<BunchLink> uplinks;
//    TileSet set;

    int size = 0;
    int size_regs = 0;
    int size_regs_own = 0;
    int size_comb = 0;
    int size_comb_own = 0;
};


}

//   │clk
// ┌─▼──┐                ┌──┐  ┌──┐                               │clk
// │┌──┐│        ────────┤  │  │  │◄───────────────┐         reg┌─▼──┐
// ││  ││        ────────┤  │◄─┤  │  ┌──┐          │┌──┐        │┌──┐│
// ││  ││        ────────┤  │  │  │◄┐│  │◄─────────┴┤  │◄──────┬┤│  ││
// ││  ││        ────────┤  │  └──┘ ├┤  │           └──┘       ││└──┘│
// ││  ││        ────────┤  │◄──────┘│  │◄─────────────────────┘└────┘
// ││  ││        ────────┤  │        │  │             │clk
// ││  ││        ────────┤  │        │  │    reg_out┌─▼──┐
// │└──┘│               ┌┤  │◄──────┐│  │           │┌──┐│
// └────┘               │└──┘ ┌─────┼┤  │◄──────────┤│  ││
//                 ┌──┐ │┌──┐ │┌──┐ ││  │           │└──┘│
// ┌───┐ "reg"     │  │◄┘│  │◄┘│  │◄┘│  │           └────┘ "reg"┌───┐
// │ │ │◄──────────┤  │◄─┤  │◄┬┤  │  │  │◄──────────────────────┤ ▲ │
// └─▼─┘           │  │  └──┘ ││  │◄┐│  │           ┌────┐      └─┴─┘
//  OBUF           │  │◄──────┤└──┘ ├┤  │◄──────────┤┌──┐│       IBUF
// ┌────┐          │  │◄┐┌──┐ │┌──┐ ││  │◄──────────┤│  ││
// │┌──┐│          └──┘ ││  │◄┘│  │◄┘└──┘           ││  ││
// │└──┘│       ────────┼┤  │◄─┤  │◄────────────────┤│  ││
// └▲───┘               ││  │◄┐│  │◄┐┌──┐           ││  ││
// ┌────┐               │└──┘ │└──┘ ││  │◄──────────┤│  ││
// │┌──┐│       ────────┤     │┌──┐ ├┤  │◄──────────┤│  ││
// │└──┘│       ────────┤     ││  │◄┘│  │◄──────────┤│  ││
// └▲───┘               │     └┤  │  └──┘           ││  ││
// ┌────┐               │      │  │◄────────────────┤└──┘│
// │┌──┐│       ────────┤      └──┘          reg_out└─▲──┘
// │└──┘│       ────────┘                             │clk
// └▲───┘
//  │clk
