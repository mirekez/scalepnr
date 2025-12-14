#include "Tech.h"
#include "Device.h"
#include "Timings.h"
#include "RtlFormat.h"
#include "PrintDesign.h"
#include "getInsts.h"

#include <vector>
#include <functional>

using namespace technology;

CombDelays Tech::comb_delays;
std::multimap<std::string,std::string> Tech::clocked_ports;
std::multimap<std::string,std::string> Tech::buffers_ports;

Tech& Tech::current()
{
    static bool inited = false;
    static Tech tech;
    if (!inited) {
        tech.init();
        inited = true;
    }
    return tech;
}

void Tech::recursivePrintTimingReport(clk::TimingPath& path, unsigned limit, int level)
{
    std::vector<clk::TimingPath*> paths;
    paths.reserve(path.sub_paths.size());
    for (auto& sub_path : path.sub_paths) {
        if (!sub_path.data_output && !sub_path.precalculated) {
            continue;
        }
        paths.push_back(&sub_path);
    }

    sort(paths.begin(), paths.end(), [](clk::TimingPath* a, clk::TimingPath* b) { return a->max_setup_time > b->max_setup_time; });

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

        if (sub_path->precalculated) {
            if (sub_path->precalculated->max_length < (int)limit) {
                std::print("*<- '{}'({})::: {:.3f}/{:.3f} ns, fanout: {}, fanin: {}", sub_path->precalculated->data_output->inst_ref->makeName(),
                    sub_path->precalculated->data_output->inst_ref->cell_ref->type, sub_path->precalculated->max_setup_time, sub_path->precalculated->min_setup_time,
                    (static_cast<Referable<rtl::Conn>*>(sub_path->precalculated->data_output))->getPeers().size(), sub_path->precalculated->sub_paths.size());
                if (sub_path->precalculated->sub_paths.size() > 1) {
                    std::print(" :");
                }
                recursivePrintTimingReport(*sub_path->precalculated, limit, level + 1);
            }
            else {
                std::print(" <- '{}'({}) ...(depth {}/{} is hidden)::: {:.3f}/{:.3f} ns, fanout: {}, fanin: {}", sub_path->precalculated->data_output->inst_ref->makeName(),
                    sub_path->precalculated->data_output->inst_ref->cell_ref->type, sub_path->precalculated->max_length, sub_path->precalculated->min_length,
                    sub_path->precalculated->max_setup_time, sub_path->precalculated->min_setup_time, (static_cast<Referable<rtl::Conn>*>(sub_path->precalculated->data_output))->getPeers().size(),
                    sub_path->precalculated->sub_paths.size());
            }
        }
        else {
            std::print(" <- '{}'({})::: {:.3f}/{:.3f} ns, fanout: {}, fanin: {}", sub_path->data_output->inst_ref->makeName(), sub_path->data_output->inst_ref->cell_ref->type,
                sub_path->max_setup_time, sub_path->min_setup_time, (static_cast<Referable<rtl::Conn>*>(sub_path->data_output))->getPeers().size(), sub_path->sub_paths.size());
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

void Tech::prepareTimingLists()
{
    timings.makeTimingsList(design, clocks);
}

void Tech::estimateTimings(unsigned limit_paths, unsigned limit_rows)
{
    timings.calculateTimings();
    for (auto& conns : timings.clocked_inputs) {
        std::print("\nclock: {}", conns.first->name);

        std::vector<clk::Timings::TimingInfo*> infos;
        infos.reserve(conns.second.size());
        for (auto& info : conns.second) {
            if (!info.path.data_output) {
                continue;
            }
            infos.push_back(&info);
        }

        sort(infos.begin(), infos.end(), [](clk::Timings::TimingInfo* a, clk::Timings::TimingInfo* b) { return a->path.max_setup_time > b->path.max_setup_time; });

        unsigned cnt = 0;
        for (auto& info : infos) {
            if (++cnt == limit_paths) {
                break;
            }
            std::print("\nconn: '{}' ('{}')::: {:.3f}/{:.3f}ns, length: {}/{}", info->data_in->makeName(), info->data_in->inst_ref->cell_ref->type,
                info->path.max_setup_time, info->path.min_setup_time, info->path.max_length, info->path.min_length);
            recursivePrintTimingReport(info->path, limit_rows);
        }
        if (infos.size() > limit_paths) {
            std::print("\n...");
        }
    }
}

void Tech::openDesign()
{
    std::print("\nOpening design...");
    estimate.clocks = &clocks;
    estimate.estimateDesign(design);
    estimate.printBunches();
}

void Tech::placeDesign()
{
    std::print("\nPlacing design...");
    outline.placeIOBs(estimate.data_outs, assignments);
    outline.optimizeOutline(estimate.data_outs);
    place.placeDesign(estimate.data_outs);
}

void Tech::routeDesign()
{
    std::print("\nRouting design...");
    route.routeDesign(estimate.data_outs);
}

void Tech::printDesign(std::string& inst_name, int limit)
{
    rtl::PrintDesign printer;
    printer.tech = this;
    printer.limit = limit;

    if (inst_name == "*") {
        for (auto& out : estimate.data_outs) {
            printer.print(out.reg);
        }
    }
    else {
        std::vector<rtl::Inst*> insts;
        std::vector<rtl::instFilter> filters;
        filters.emplace_back(rtl::instFilter{});
        filters.back().blackbox = true;
        filters.back().regexp = true;
        filters.back().name = inst_name;

        rtl::getInsts(&insts, filters, &design.top);
        for (auto& inst : insts) {
            printer.print(inst);
            break;
        }
    }
}

void Tech::loadDesign(const std::string& filename, const std::string& top_module)
{
    std::print("\nLoading design from '{}' ('{}')...", filename, top_module);
    rtl::Design& rtl = Tech::current().design;
    RtlFormat rtl_format;
    rtl_format.loadFromJson(filename, &rtl);
    rtl.build(top_module);
    rtl.printReport();
}

//void Tech::printDesign(std::string& inst_name, int limit)
//{
//}

void Tech::init()
{
//    design.tech = this;
    clocks.tech = this;
    timings.tech = this;
    estimate.tech = this;
    outline.tech = this;
    place.tech = this;
    route.tech = this;
    route.fpga = &fpga::Device::current();

    comb_delays = {{  // this is just for RTL timing estimation testing
        {"INV", {1, {0.05,0.05}}},
        {"LUT2", {2, {0.08,0.08}}},
        {"LUT3", {3, {0.1,0.1,0.1}}},
        {"LUT4", {4, {0.1,0.1,0.1,0.1}}},
        {"LUT5", {5, {0.1,0.1,0.1,0.1,0.1}}},
        {"LUT6", {6, {0.1,0.1,0.1,0.1,0.1,0.1}}},
        {"LUT6_2", {6, {0.1,0.1,0.1,0.1,0.1,0.1, 0.1,0.1,0.1,0.1,0.1,0.1}}},
        {"MUXF7", {3, {0.02,0.02,0.02}}},
        {"MUXF8", {3, {0.02,0.02,0.02}}},
        {"CARRY4", {10, {0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02, 0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02, 0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02, 0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,
                         0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02, 0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02, 0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02, 0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,}}}
    }};

    clocked_ports = {  // yosys RTL ports
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

    buffers_ports = {
    {"BUFG", "O"},
    {"IBUF", "O"},
    {"OBUF", "O"},
    };

    fpga::TileType tile0{"Tile0", 123};
    fpga::TileType tile1{"Tile1", 123};
    fpga::Device::current().tile_types.push_back(tile1);
}

void technology::readTechMap(std::string maptext, TechMap& map)
{
    map.clear();
    std::stringstream ss(maptext);
    std::string line;
    while (std::getline(ss, line, '\n')) {
        map.resize(map.size()+1);
        auto& lineref = map.back();
        std::string expr;
        std::stringstream ss1(line);
        while (std::getline(ss1, expr, ';')) {
            lineref.resize(lineref.size()+1);
            auto& exprref = lineref.back();
            std::string equal;
            std::stringstream ss2(expr);
            while (std::getline(ss2, equal, '=')) {
                exprref.resize(exprref.size()+1);
                auto& equalref = exprref.back();
                std::string token;
                std::stringstream ss3(equal);
                while (std::getline(ss3, token, ':')) {
                    equalref.resize(equalref.size()+1);
                    auto& tokenref = equalref.back();
                    std::string part;
                    std::stringstream ss4(token);
                    while (std::getline(ss4, part, ',')) {
                        tokenref.emplace_back(part);
                        std::print("\nemplacing {} {} {} {} {}", line, expr, equal, token, part);
                    }
                }
            }
        }
    }
}

