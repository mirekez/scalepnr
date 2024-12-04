
#include "XC7Tech.h"
#include "Device.h"

#include <vector>
#include <functional>

XC7Tech& XC7Tech::current()
{
    static XC7Tech tech;
    return tech;
}

void XC7Tech::init()
{
    gear::TileType tile0{"CLBLL", 123};
    tile0.bells.push_back(BelType{"SLICE0L_D5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE0L_D6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile0.bells.push_back(BelType{"SLICE0L_C5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE0L_C6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile0.bells.push_back(BelType{"SLICE0L_B5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE0L_B6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile0.bells.push_back(BelType{"SLICE0L_A5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE0L_A6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});

    tile0.bells.push_back(BelType{"SLICE0L_D5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_DCY0", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_F7BMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_C5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_CCY0", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_F8MUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_B5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_BCY0", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_F7AMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_A5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE0L_ACY0", BelType::MUX});

    tile0.bells.push_back(BelType{"SLICE0L_PRECYINIT", BelType::MUX});

    tile0.bells.push_back(BelType{"SLICE0L_CEUSEDMUX", BelType::MUX, 1});
    tile0.bells.push_back(BelType{"SLICE0L_SRUSEDMUX", BelType::MUX, 1});

    tile0.bells.push_back(BelType{"SLICE0L_DOUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_DFFMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_COUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_CFFMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_BOUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_BFFMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_AOUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE0L_AFFMUX", BelType::MUX, 5});

    tile0.bells.push_back(BelType{"SLICE0L_D5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_DFF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_C5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_CFF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_B5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_BFF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_A5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE0L_AFF", BelType::FF, 4});

    tile0.bells.push_back(BelType{"SLICE0L_CLKINV", BelType::CLKINV, 2});

    tile0.bells.push_back(BelType{"SLICE1L_D5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE1L_D6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile0.bells.push_back(BelType{"SLICE1L_C5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE1L_C6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile0.bells.push_back(BelType{"SLICE1L_B5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE1L_B6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile0.bells.push_back(BelType{"SLICE1L_A5LUT", BelType::LUT, 5});
    tile0.bells.push_back(BelType{"SLICE1L_A6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});

    tile0.bells.push_back(BelType{"SLICE1L_D5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_DCY0", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_F7BMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_C5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_CCY0", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_F8MUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_B5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_BCY0", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_F7AMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_A5FFMUX", BelType::MUX});
    tile0.bells.push_back(BelType{"SLICE1L_ACY0", BelType::MUX});

    tile0.bells.push_back(BelType{"SLICE1L_PRECYINIT", BelType::MUX});

    tile0.bells.push_back(BelType{"SLICE1L_CEUSEDMUX", BelType::MUX, 1});
    tile0.bells.push_back(BelType{"SLICE1L_SRUSEDMUX", BelType::MUX, 1});

    tile0.bells.push_back(BelType{"SLICE1L_DOUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_DFFMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_COUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_CFFMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_BOUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_BFFMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_AOUTMUX", BelType::MUX, 5});
    tile0.bells.push_back(BelType{"SLICE1L_AFFMUX", BelType::MUX, 5});

    tile0.bells.push_back(BelType{"SLICE1L_D5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_DFF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_C5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_CFF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_B5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_BFF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_A5FF", BelType::FF, 4});
    tile0.bells.push_back(BelType{"SLICE1L_AFF", BelType::FF, 4});

    tile0.bells.push_back(BelType{"SLICE1L_CLKINV", BelType::CLKINV, 2});
    gear::Device::current().tile_types.push_back(tile0);

    /////////////////////////////////////

    gear::TileType tile1{"CLBLM", 123};
    tile1.bells.push_back(BelType{"SLICE0L_D5LUT", BelType::LUT, 5});
    tile1.bells.push_back(BelType{"SLICE0L_D6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile1.bells.push_back(BelType{"SLICE0L_C5LUT", BelType::LUT, 5});
    tile1.bells.push_back(BelType{"SLICE0L_C6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile1.bells.push_back(BelType{"SLICE0L_B5LUT", BelType::LUT, 5});
    tile1.bells.push_back(BelType{"SLICE0L_B6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});
    tile1.bells.push_back(BelType{"SLICE0L_A5LUT", BelType::LUT, 5});
    tile1.bells.push_back(BelType{"SLICE0L_A6LUT", BelType::LUT, 1, BelType::SHARE_PREV_INPUTS});

    tile1.bells.push_back(BelType{"SLICE0L_D5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_DCY0", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_F7BMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_C5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_CCY0", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_F8MUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_B5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_BCY0", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_F7AMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_A5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE0L_ACY0", BelType::MUX});

    tile1.bells.push_back(BelType{"SLICE0L_PRECYINIT", BelType::MUX});

    tile1.bells.push_back(BelType{"SLICE0L_CEUSEDMUX", BelType::MUX, 1});
    tile1.bells.push_back(BelType{"SLICE0L_SRUSEDMUX", BelType::MUX, 1});

    tile1.bells.push_back(BelType{"SLICE0L_DOUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_DFFMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_COUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_CFFMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_BOUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_BFFMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_AOUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE0L_AFFMUX", BelType::MUX, 5});

    tile1.bells.push_back(BelType{"SLICE0L_D5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_DFF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_C5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_CFF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_B5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_BFF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_A5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE0L_AFF", BelType::FF, 4});

    tile1.bells.push_back(BelType{"SLICE0L_CLKINV", BelType::CLKINV, 2});

    tile1.bells.push_back(BelType{"SLICE1M_D5LUT", BelType::LUTRAM, 13});
    tile1.bells.push_back(BelType{"SLICE1M_D6LUT", BelType::LUTRAM, 5, BelType::SHARE_PREV_INPUTS, 2});
    tile1.bells.push_back(BelType{"SLICE1M_C5LUT", BelType::LUTRAM, 13});
    tile1.bells.push_back(BelType{"SLICE1M_C6LUT", BelType::LUTRAM, 5, BelType::SHARE_PREV_INPUTS, 2});
    tile1.bells.push_back(BelType{"SLICE1M_B5LUT", BelType::LUTRAM, 13});
    tile1.bells.push_back(BelType{"SLICE1M_B6LUT", BelType::LUTRAM, 5, BelType::SHARE_PREV_INPUTS, 2});
    tile1.bells.push_back(BelType{"SLICE1M_A5LUT", BelType::LUTRAM, 13});
    tile1.bells.push_back(BelType{"SLICE1M_A6LUT", BelType::LUTRAM, 5, BelType::SHARE_PREV_INPUTS, 2});

    tile1.bells.push_back(BelType{"SLICE1M_CDI1MUX", BelType::MUX, 3});
    tile1.bells.push_back(BelType{"SLICE1M_BDI1MUX", BelType::MUX, 3});
    tile1.bells.push_back(BelType{"SLICE1M_ADI1MUX", BelType::MUX, 3});

    tile1.bells.push_back(BelType{"SLICE1M_D5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_DCY0", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_F7BMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_C5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_CCY0", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_F8MUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_B5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_BCY0", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_F7AMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_A5FFMUX", BelType::MUX});
    tile1.bells.push_back(BelType{"SLICE1M_ACY0", BelType::MUX});

    tile1.bells.push_back(BelType{"SLICE1M_PRECYINIT", BelType::MUX});

    tile1.bells.push_back(BelType{"SLICE1M_CEUSEDMUX", BelType::MUX, 1});
    tile1.bells.push_back(BelType{"SLICE1M_SRUSEDMUX", BelType::MUX, 1});

    tile1.bells.push_back(BelType{"SLICE1M_DOUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_DFFMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_COUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_CFFMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_BOUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_BFFMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_AOUTMUX", BelType::MUX, 5});
    tile1.bells.push_back(BelType{"SLICE1M_AFFMUX", BelType::MUX, 5});

    tile1.bells.push_back(BelType{"SLICE1M_D5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_DFF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_C5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_CFF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_B5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_BFF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_A5FF", BelType::FF, 4});
    tile1.bells.push_back(BelType{"SLICE1M_AFF", BelType::FF, 4});

    tile1.bells.push_back(BelType{"SLICE1M_CLKINV", BelType::CLKINV, 2});
    gear::Device::current().tile_types.push_back(tile1);

    gear::TileType tile2{"LIOI3_TBYTESRC", 123};
    tile2.bells.push_back(BelType{"DINV", BelType::MUX, 2});
    tile2.bells.push_back(BelType{"SLICE0_ZHOLD_DELAY", BelType::ZHOLD_DELAY, 1, BelType::SIMPLE, 2});
    tile2.bells.push_back(BelType{"ZHOLD_FABRIC_INV", BelType::MUX, 2});
    tile2.bells.push_back(BelType{"IDEL", BelType::MUX, 3});
    tile2.bells.push_back(BelType{"D2OBYP_SRC", BelType::DMUX, 3});
    tile2.bells.push_back(BelType{"D2OBYP_SEL", BelType::DMUX, 2});
    tile2.bells.push_back(BelType{"IFFDELMUXE3", BelType::DMUX, 3});
    tile2.bells.push_back(BelType{"D2OFFBYP_SRC", BelType::DMUX, 3});
    tile2.bells.push_back(BelType{"D2OFFBYP_SEL", BelType::DMUX, 2});
    tile2.bells.push_back(BelType{"IMUX", BelType::DMUX, 2});
    tile2.bells.push_back(BelType{"IFFMUX", BelType::DMUX, 2});
    tile2.bells.push_back(BelType{"IFF", BelType::IFF, 6, BelType::SIMPLE, 2});
    tile2.bells.push_back(BelType{"CLKBINV", BelType::CLKINV});
    tile2.bells.push_back(BelType{"CLKINV", BelType::CLKINV});

    tile2.bells.push_back(BelType{"IDELAYE2", BelType::IDELAY, 14, BelType::SIMPLE, 6});
    tile2.bells.push_back(BelType{"IDATAININV", BelType::MUX, 2, BelType::SIMPLE, 1});
    tile2.bells.push_back(BelType{"DATAININV", BelType::MUX, 2, BelType::SIMPLE, 1});
    tile2.bells.push_back(BelType{"CINV", BelType::MUX, 2, BelType::SIMPLE, 1});

    gear::TileType tile3{"LIOI3_TBYTESRC", 123};
    tile3.bells.push_back(BelType{"TFF", BelType::TFF, 6});
    tile3.bells.push_back(BelType{"OUTFF", BelType::TFF, 6});
    tile3.bells.push_back(BelType{"TMUX", BelType::TFF, 2});
    tile3.bells.push_back(BelType{"MISR", BelType::MISR, 1});
    tile3.bells.push_back(BelType{"T2INV", BelType::INV, 1});
    tile3.bells.push_back(BelType{"T1INV", BelType::INV, 1});
    tile3.bells.push_back(BelType{"D2INV", BelType::INV, 1});
    tile3.bells.push_back(BelType{"D1INV", BelType::INV, 1});
    tile3.bells.push_back(BelType{"CLKINV", BelType::CLKINV, 1});
    tile3.bells.push_back(BelType{"O_ININV", BelType::ININV, 1});

//    gear::TileType tile4{"PAD", 123, {{{2,0},{10,10}}}};
//    tile4.bells.push_back(BelType{"PAD", BelType::PAD, 0});
//    tile4.bells.push_back(BelType{"OUTBUF", BelType::OUTBUF, 2});
//    tile4.bells.push_back(BelType{"INBUF_EN", BelType::INBUF, 0});
//    tile4.bells.push_back(BelType{"OUTMUX", BelType::MUX, 2});
//    tile4.bells.push_back(BelType{"TINMUX", BelType::MUX, 2});
//    tile4.bells.push_back(BelType{"OINMUX", BelType::MUX, 2});
//    tile4.bells.push_back(BelType{"INTERMDISABLE_SEL", BelType::MUX, 2});
//    tile4.bells.push_back(BelType{"IBUFDISABLE_SEL", BelType::MUX, 2});

}

void XC7Tech::recursivePrintTimingReport(rtl::Timing& path, unsigned limit, int level)
{
    std::vector<rtl::Timing*> paths;
    paths.reserve(path.sub_paths.size());
    for (auto& sub_path : path.sub_paths) {
        if (!sub_path.data_output && !sub_path.precalculated.ref) {
            continue;
        }
        paths.push_back(&sub_path);
    }

    sort(paths.begin(), paths.end(), [](rtl::Timing* a, rtl::Timing* b) { return a->max_setup_time > b->max_setup_time; });

    unsigned cnt = 0;
    for (auto& sub_path : paths) {
        if (++cnt == limit) {
            break;
        }

        if (path.sub_paths.size() > 1) {
            std::print("\n");
            for (int i=0; i < level + 1; ++i) {
                std::print("  ");
            }
        }

        if (sub_path->precalculated.ref) {
            if (sub_path->precalculated->max_length < (int)limit) {
                std::print("*<- '{}'({})::: {:.3f}/{:.3f}ns", sub_path->precalculated->data_output->inst_ref->makeName(),
                    sub_path->precalculated->data_output->inst_ref->cell_ref->type, sub_path->precalculated->max_setup_time, sub_path->precalculated->min_setup_time);
                if (sub_path->precalculated->sub_paths.size() > 1) {
                    std::print(" :");
                }
                recursivePrintTimingReport(*sub_path->precalculated.ref, limit, level + 1);
            }
            else {
                std::print(" <- '{}'({}) ...(depth {}/{} is hidden)::: {:.3f}/{:.3f}ns", sub_path->precalculated->data_output->inst_ref->makeName(),
                    sub_path->precalculated->data_output->inst_ref->cell_ref->type, sub_path->precalculated->max_length, sub_path->precalculated->min_length,
                    sub_path->precalculated->max_setup_time, sub_path->precalculated->min_setup_time);
            }
        }
        else {
            std::print(" <- '{}'({})::: {:.3f}/{:.3f}ns", sub_path->data_output->inst_ref->makeName(),
                sub_path->data_output->inst_ref->cell_ref->type, sub_path->max_setup_time, sub_path->min_setup_time);
            if (sub_path->sub_paths.size() > 1) {
                std::print(" :");
            }
            recursivePrintTimingReport(*sub_path, limit, level + 1);
        }
    }
    if (paths.size() > limit) {
        std::print("\n");
        for (int i=0; i < level + 1; ++i) {
            std::print("  ");  //??
        }
        std::print(" ...");
    }
}

void XC7Tech::prepareTimingLists()
{
    timings.makeTimingsList(clocked_ports, buffers_ports);
}

void XC7Tech::estimateTimings(unsigned limit_paths, unsigned limit_rows)
{
    timings.calculateTimings(comb_delays);
    for (auto& conns : timings.clocked_inputs) {
        std::print("\nclock: {}", conns.first->name);

        std::vector<clocks::Timings::TimingInfo*> infos;
        infos.reserve(conns.second.size());
        for (auto& info : conns.second) {
            if (!info.path.data_output) {
                continue;
            }
            infos.push_back(&info);
        }

        sort(infos.begin(), infos.end(), [](clocks::Timings::TimingInfo* a, clocks::Timings::TimingInfo* b) { return a->path.max_setup_time > b->path.max_setup_time; });

        unsigned cnt = 0;
        for (auto& info : infos) {
            if (++cnt == limit_paths) {
                break;
            }
            std::print("\nconn: '{}' ('{}')::: {:.3f}/{:.3f}ns, length: {}/{}", info->data_input->makeName(), info->data_input->inst_ref->cell_ref->type,
                info->path.max_setup_time, info->path.min_setup_time, info->path.max_length, info->path.min_length);
            recursivePrintTimingReport(info->path, limit_rows);
        }
        if (infos.size() > limit_paths) {
            std::print("\n...");
        }
    }
}

std::multimap<std::string,std::string> XC7Tech::clocked_ports = {
    {"FD", "C"},
    {"FDCE", "C"},
    {"FDCE_1", "C"},
    {"FDPE", "C"},
    {"FDPE_1", "C"},
    {"FDRE", "C"},
    {"FDRE_1", "C"},
    {"FDSE", "C"},
    {"FDSE_1", "C"},
    {"FDRSE", "C"},
    {"FDRSE_1", "C"},
    {"FDDRCPE", "C0"},
    {"FDDRCPE", "C1"},
    {"FDDRRSE", "C0"},
    {"FDDRRSE", "C1"},
    {"SRL16E", "CLK"},
    {"SRLC16", "CLK"},
    {"SRLC16E", "CLK"},
    {"SRL32E", "CLK"},
    {"RAM128X1D", "WCLK"},
    {"RAM128X1S", "WCLK"},
    {"RAM128X1S_1", "WCLK"},
    {"RAM16X1D", "WCLK"},
    {"RAM16X1D_1", "WCLK"},
    {"RAM16X1S", "WCLK"},
    {"RAM16X1S_1", "WCLK"},
    {"RAM16X2S", "WCLK"},
    {"RAM16X4S", "WCLK"},
    {"RAM16X8S", "WCLK"},
    {"RAM256X1D", "WCLK"},
    {"RAM256X1S", "WCLK"},
    {"RAM32M", "WCLK"},
    {"RAM32M16", "WCLK"},
    {"RAM32X16DR8", "WCLK"},
    {"RAM32X1D", "WCLK"},
    {"RAM32X1D_1", "WCLK"},
    {"RAM32X1S", "WCLK"},
    {"RAM32X1S_1", "WCLK"},
    {"RAM32X2S", "WCLK"},
    {"RAM32X4S", "WCLK"},
    {"RAM32X8S", "WCLK"},
    {"RAM512X1S", "WCLK"},
    {"RAM64M", "WCLK"},
    {"RAM64M8", "WCLK"},
    {"RAM64X1D", "WCLK"},
    {"RAM64X1D_1", "WCLK"},
    {"RAM64X1S", "WCLK"},
    {"RAM64X1S_1", "WCLK"},
    {"RAM64X2S", "WCLK"},
    {"RAM64X8SW", "WCLK"},
    {"RAMB16", "CLKA"},
    {"RAMB16", "CLKB"},
    {"RAMB16BWER", "CLKA"},
    {"RAMB16BWER", "CLKB"},
    {"RAMB16BWE_S18", "CLK"},
    {"RAMB16BWE_S18_S18", "CLKA"},
    {"RAMB16BWE_S18_S18", "CLKB"},
    {"RAMB16BWE_S18_S9", "CLKA"},
    {"RAMB16BWE_S18_S9", "CLKB"},
    {"RAMB16BWE_S36", "CLK"},
    {"RAMB16BWE_S36_S18", "CLKA"},
    {"RAMB16BWE_S36_S18", "CLKB"},
    {"RAMB16BWE_S36_S36", "CLKA"},
    {"RAMB16BWE_S36_S36", "CLKB"},
    {"RAMB16BWE_S36_S9", "CLKA"},
    {"RAMB16BWE_S36_S9", "CLKB"},
    {"RAMB16_S1", "CLK"},
    {"RAMB16_S18", "CLK"},
    {"RAMB16_S18_S18", "CLKA"},
    {"RAMB16_S18_S18", "CLKB"},
    {"RAMB16_S18_S36", "CLKA"},
    {"RAMB16_S18_S36", "CLKB"},
    {"RAMB16_S1_S1", "CLKA"},
    {"RAMB16_S1_S1", "CLKB"},
    {"RAMB16_S1_S18", "CLKA"},
    {"RAMB16_S1_S18", "CLKB"},
    {"RAMB16_S1_S2", "CLKA"},
    {"RAMB16_S1_S2", "CLKB"},
    {"RAMB16_S1_S36", "CLKA"},
    {"RAMB16_S1_S36", "CLKB"},
    {"RAMB16_S1_S4", "CLKA"},
    {"RAMB16_S1_S4", "CLKB"},
    {"RAMB16_S1_S9", "CLKA"},
    {"RAMB16_S1_S9", "CLKB"},
    {"RAMB16_S2", "CLK"},
    {"RAMB16_S2_S18", "CLKA"},
    {"RAMB16_S2_S18", "CLKB"},
    {"RAMB16_S2_S2", "CLKA"},
    {"RAMB16_S2_S2", "CLKB"},
    {"RAMB16_S2_S36", "CLKA"},
    {"RAMB16_S2_S36", "CLKB"},
    {"RAMB16_S2_S4", "CLKA"},
    {"RAMB16_S2_S4", "CLKB"},
    {"RAMB16_S2_S9", "CLKA"},
    {"RAMB16_S2_S9", "CLKB"},
    {"RAMB16_S36", "CLK"},
    {"RAMB16_S36_S36", "CLKA"},
    {"RAMB16_S36_S36", "CLKB"},
    {"RAMB16_S4", "CLK"},
    {"RAMB16_S4_S18", "CLKA"},
    {"RAMB16_S4_S18", "CLKB"},
    {"RAMB16_S4_S36", "CLKA"},
    {"RAMB16_S4_S36", "CLKB"},
    {"RAMB16_S4_S4", "CLKA"},
    {"RAMB16_S4_S4", "CLKB"},
    {"RAMB16_S4_S9", "CLKA"},
    {"RAMB16_S4_S9", "CLKB"},
    {"RAMB16_S9", "CLK"},
    {"RAMB16_S9_S18", "CLKA"},
    {"RAMB16_S9_S18", "CLKB"},
    {"RAMB16_S9_S36", "CLKA"},
    {"RAMB16_S9_S36", "CLKB"},
    {"RAMB16_S9_S9", "CLKA"},
    {"RAMB16_S9_S9", "CLKB"},
    {"RAMB18", "CLKA"},
    {"RAMB18", "CLKB"},
    {"RAMB18E1", "CLKARDCLK"},
    {"RAMB18E1", "CLKAWRCLK"},
    {"RAMB18E2", "CLKARDCLK"},
    {"RAMB18E2", "CLKAWRCLK"},
    {"RAMB18SDP", "RDCLK"},
    {"RAMB18SDP", "WRCLK"},
    {"RAMB32_S64_ECC", "RDCLK"},
    {"RAMB32_S64_ECC", "WRCLK"},
    {"RAMB36", "CLKA"},
    {"RAMB36", "CLKB"},
    {"RAMB36E1", "CLKARDCLK"},
    {"RAMB36E1", "CLKAWRCLK"},
    {"RAMB36E2", "CLKARDCLK"},
    {"RAMB36E2", "CLKAWRCLK"},
    {"RAMB36SDP", "RDCLK"},
    {"RAMB36SDP", "WRCLK"},
    {"RAMB4_S1", "CLK"},
    {"RAMB4_S16", "CLK"},
    {"RAMB4_S16_S16", "CLKA"},
    {"RAMB4_S16_S16", "CLKB"},
    {"RAMB4_S1_S1", "CLKA"},
    {"RAMB4_S1_S1", "CLKB"},
    {"RAMB4_S1_S16", "CLKA"},
    {"RAMB4_S1_S16", "CLKB"},
    {"RAMB4_S1_S2", "CLKA"},
    {"RAMB4_S1_S2", "CLKB"},
    {"RAMB4_S1_S4", "CLKA"},
    {"RAMB4_S1_S4", "CLKB"},
    {"RAMB4_S1_S8", "CLKA"},
    {"RAMB4_S1_S8", "CLKB"},
    {"RAMB4_S2", "CLK"},
    {"RAMB4_S2_S16", "CLKA"},
    {"RAMB4_S2_S16", "CLKB"},
    {"RAMB4_S2_S2", "CLKA"},
    {"RAMB4_S2_S2", "CLKB"},
    {"RAMB4_S2_S4", "CLKA"},
    {"RAMB4_S2_S4", "CLKB"},
    {"RAMB4_S2_S8", "CLKA"},
    {"RAMB4_S2_S8", "CLKB"},
    {"RAMB4_S4", "CLK"},
    {"RAMB4_S4_S16", "CLKA"},
    {"RAMB4_S4_S16", "CLKB"},
    {"RAMB4_S4_S4", "CLKA"},
    {"RAMB4_S4_S4", "CLKB"},
    {"RAMB4_S4_S8", "CLKA"},
    {"RAMB4_S4_S8", "CLKB"},
    {"RAMB4_S8", "CLK"},
    {"RAMB4_S8_S16", "CLKA"},
    {"RAMB4_S8_S16", "CLKB"},
    {"RAMB4_S8_S8", "CLKA"},
    {"RAMB4_S8_S8", "CLKB"},
    {"RAMB8BWER", "CLKARDCLK"},
    {"RAMB8BWER", "CLKAWRCLK"},
};

std::multimap<std::string,std::string> XC7Tech::buffers_ports = {
    {"BUFG", "O"},
    {"IBUF", "O"},
};

std::map<std::string,std::pair<int,std::vector<double>>> XC7Tech::comb_delays = {
    {"LUT2", {2, {0.1,0.1}}},
    {"LUT3", {3, {0.1,0.1,0.1}}},
    {"LUT4", {4, {0.1,0.1,0.1,0.1}}},
    {"LUT5", {5, {0.1,0.1,0.1,0.1,0.1}}},
    {"LUT6", {6, {0.1,0.1,0.1,0.1,0.1,0.1}}},
    {"LUT6_2", {6, {0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1}}},
};
