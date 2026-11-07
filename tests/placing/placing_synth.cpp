#include "Tech.h"
#include "Device.h"
#include "Element.h"
#include "Tile.h"
#include "pnr.h"
#include "json/json.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

namespace {
void require(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}
using Clock = std::chrono::steady_clock;
struct Fixture {
    technology::Tech tech;
    fpga::TileType tile_type;
    Referable<rtl::Module>* top;
    Referable<rtl::Module>* primitive;
    std::list<Referable<rtl::Cell>> cells;
    std::vector<rtl::Inst*> core;
    std::map<int, Referable<rtl::Conn>*> drivers;
    std::vector<std::pair<Referable<rtl::Conn>*, int>> sinks;
    std::unordered_map<rtl::Inst*, std::pair<fpga::Coord,int>> fixed;
    Json::Value metrics{Json::objectValue};
    size_t expected_endpoints = 0, outlined = 0, packed = 0, sorted = 0, swapped = 0;
    int side = 0;
    Clock::time_point last_stage = Clock::now();

    static int bit(const Json::Value& value) {
        if (value.isInt() || value.isUInt()) return value.asInt();
        auto text = value.asString();
        require(text == "0" || text == "1", "unknown/high-impedance bit in synthesized design");
        return text == "0" ? -2 : -3;
    }

    Referable<rtl::Inst>& cell(const std::string& name, const std::string& type,
                             const std::vector<std::tuple<std::string,bool,int>>& ports) {
        auto& c = cells.emplace_back(); c.name = name; c.type = type; c.module_ref.set(primitive);
        c.ports.reserve(ports.size());
        auto& inst = tech.design.top.insts.emplace_back();
        inst.cell_ref.set(&c); inst.parent_ref.set(&tech.design.top);
        inst.depth = 1; inst.pos = -1; inst.coord = {-1,-1}; inst.outline={-1,-1,false};
        inst.conns.reserve(ports.size());
        int inputs = 0, outputs = 0;
        for (const auto& [name, input, id] : ports) {
            auto& port = c.ports.emplace_back();
            port.name=name; port.type=input ? rtl::Port::PORT_IN : rtl::Port::PORT_OUT;
            port.index=input ? inputs++ : outputs++; port.designator=id;
            auto& conn = inst.conns.emplace_back(); conn.port_ref.set(&port); conn.inst_ref.set(&inst);
            if (input) sinks.emplace_back(&conn,id);
            else {
                require(!drivers.contains(id), "multiple drivers for net " + std::to_string(id));
                drivers[id]=&conn;
            }
        }
        inst.cnt_inputs=inputs; inst.cnt_outputs=outputs;
        return inst;
    }

    void device(const Json::Value& module) {
        size_t luts=0, small=0, regs=0;
        for (const auto& c : module["cells"]) {
            if (c["type"] == "$lut") {
                if (c["connections"]["A"].size()==1) ++small; else ++luts;
            } else ++regs;
        }
        side=std::max(4, int(std::ceil(std::sqrt(std::max({luts,small,regs})/(16*.65)))));
        tile_type.name="GENERIC_LOGIC"; tile_type.type=fpga::Tile::TILE_LUTS;
        for (int i=0;i<16;++i) {
            for (auto type : {fpga::ELEMENT_LUT5,fpga::ELEMENT_LUT1,fpga::ELEMENT_FD}) {
                fpga::Element e{}; e.name=std::to_string(type)+"_"+std::to_string(i);
                e.type=type; e.bitmap_pos=i; e.elements_to_left=type;
                tile_type.elements.push_back(e);
            }
            tile_type.sites.push_back(fpga::SiteModel{.name="IO_"+std::to_string(i),.type="IO",.pos=1024+i});
        }
        auto& d=fpga::Device::current();
        d.tile_grid.clear(); d.tile_types.clear(); d.cb_types.clear(); d.pins.clear();
        d.size_width=side; d.size_height=side; d.grid_spec.size={side,side};
        // Outline's scalar LUT budget describes general logic sites. Separate
        // inverter slots cannot substitute for LUT2..4 and must not double it.
        d.cnt_luts=side*side*16; d.cnt_regs=side*side*16;
        d.tile_grid.resize(side*side);
        for (int y=0;y<side;++y) for(int x=0;x<side;++x) {
            auto& tile=d.tile_grid[y*side+x]; tile.coord={x,y}; tile.cb_coord=tile.coord;
            tile.name=tile.coord; tile.type=fpga::Tile::TILE_LUTS; tile.tile_type=&tile_type;
            tile.full_name="GENERIC_X"+std::to_string(x)+"Y"+std::to_string(y);
            for (const auto& s:tile_type.sites) { tile.sites.push_back(s.name); tile.site_types.push_back(s.type); }
        }
        for (auto& tile:d.tile_grid) tile.attached_resource_tiles={&tile};
        metrics["device_side"]=side; metrics["luts"]=Json::UInt64(luts+small);
        metrics["registers"]=Json::UInt64(regs);
    }

    void anchor(Referable<rtl::Inst>& inst, fpga::Coord coord, int pos) {
        inst.coord=coord; inst.pos=pos;
        inst.outline={float(coord.x)*10/side,float(coord.y)*10/side,true};
        fpga::Device::current().tile_grid[coord.y*side+coord.x].assign(&inst);
        fixed[&inst]={coord,pos};
    }

    Fixture(const Json::Value& module, double period) {
        device(module);
        tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
        tech.comb_delays.map["LUT1"]={1,{0.05}};
        tech.estimate.clocks=&tech.clocks;
        tech.place.place_timing.tech=&tech;
        tech.place.write_debug_images=false;
        top=&tech.design.modules.emplace_back(); top->name="top"; top->is_blackbox=false;
        primitive=&tech.design.modules.emplace_back(); primitive->name="generic_primitives";
        primitive->is_blackbox=true; primitive->parent_ref.set(top);
        tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
        tech.design.top_cell.module_ref.set(top); tech.design.top.cell_ref.set(&tech.design.top_cell);
        tech.design.top.depth=0;
        tech.design.top_cell.ports.reserve(16); tech.design.top.conns.reserve(16);
        int next=10000000;
        for (const auto& [name,id] : {std::pair{"GND",-2}, {"VCC",-3}}) {
            auto& inst=cell(name,name,{{"O",false,id}});
            inst.cell_ref->attributes["scalepnr_constant"]="1";
            inst.cell_ref->attributes["scalepnr_constant_role"]="source";
            inst.conns[0].port_ref->is_global=true;
            if (id==-2) tech.design.GND=&inst.conns[0]; else tech.design.VCC=&inst.conns[0];
        }
        size_t input_index=0, output_index=0;
        for (const auto& name:module["ports"].getMemberNames()) {
            const auto& p=module["ports"][name];
            require(p["direction"]=="input" || p["direction"]=="output", "inout unsupported");
            for (const auto& b:p["bits"]) {
                int id=bit(b);
                if (p["direction"]=="input") {
                    auto& inst=cell(name+"_"+std::to_string(input_index),"IBUF",{{"O",false,id}});
                    anchor(inst,{0,int(input_index%side)},1024+input_index/side);
                    ++input_index;
                    if (name=="clk") {
                        tech.clocks.clocks_list.emplace_back(rtl::Clock{
                            .name="clk",.conn_ptr=&inst.conns[0],.conn_name="clk.O",.period_ns=period,.duty=50});
                    }
                } else {
                    auto& inst=cell(name+"_"+std::to_string(output_index),"OBUF",{{"I",true,id},{"O",false,next++}});
                    anchor(inst,{side-1,int(output_index%side)},1024+output_index/side);
                    auto& port=tech.design.top_cell.ports.emplace_back(); port.name="out_"+std::to_string(output_index);
                    port.type=rtl::Port::PORT_OUT; port.index=output_index++; port.designator=next-1;
                    auto& conn=tech.design.top.conns.emplace_back(); conn.port_ref.set(&port); conn.inst_ref.set(&tech.design.top);
                    conn.set(&inst.conns[1]);
                }
            }
        }
        for (const auto& name:module["cells"].getMemberNames()) {
            const auto& c=module["cells"][name]; const auto original=c["type"].asString();
            const bool lut=original=="$lut";
            require(lut || original.starts_with("$_DFF_") || original.starts_with("$_DFFE_") ||
                    original.starts_with("$_SDFF_") || original.starts_with("$_SDFFE_") ||
                    original.starts_with("$_SDFFCE_"), "unsupported primitive "+original);
            std::string type=lut ? "LUT"+std::to_string(c["connections"]["A"].size()) : "FD"+original;
            if (!lut && !tech.clocked_ports.contains(type)) tech.clocked_ports.emplace(type,"C");
            std::vector<std::tuple<std::string,bool,int>> ports;
            for (const auto& pname:c["connections"].getMemberNames()) {
                int index=0;
                for (const auto& b:c["connections"][pname]) {
                    auto port_name=lut ? (pname=="A" ? "I"+std::to_string(index++) : "O") : pname;
                    bool input=c["port_directions"][pname]=="input";
                    ports.emplace_back(port_name,input,bit(b));
                    if (!lut && input && pname!="C" && bit(b)>=0) ++expected_endpoints;
                }
            }
            auto& inst=cell(name,type,ports);
            inst.cell_ref->attributes["yosys_type"]=original;
            for (const auto& parameter:c["parameters"].getMemberNames())
                inst.cell_ref->parameters[parameter]=c["parameters"][parameter].asString();
            core.push_back(&inst);
        }
        top->nets.reserve(drivers.size());
        for (const auto& [id,driver]:drivers) {
            auto& net=top->nets.emplace_back(); net.name="net_"+std::to_string(id); net.designators.push_back(id);
        }
        for (auto [sink,id]:sinks) {
            require(drivers.contains(id),"undriven net "+std::to_string(id)); sink->set(drivers.at(id));
        }
        tech.prepareTimingLists(); tech.timings.calculateTimings();
        auto before=tech.place.place_timing.analyze(tech.timings);
        require(before.endpoints==expected_endpoints,"initial timing endpoint count mismatch: "+
                std::to_string(before.endpoints)+" vs "+std::to_string(expected_endpoints));
        metrics["cells"]=Json::UInt64(core.size()); metrics["period_ns"]=period;
        metrics["expected_endpoints"]=Json::UInt64(expected_endpoints);
        metrics["stages"]=Json::Value(Json::arrayValue);
        tech.placement_stage_completed=[&](const std::string& name) { stage(name); };
    }

    void legality() {
        std::set<std::tuple<int,int,int,int>> occupied;
        for (auto& inst:tech.design.top.insts) {
            if (inst.pnr_pending) { require(!inst.tile.peer || inst.outline.fixed,"pending core cell was placed"); continue; }
            auto type=fpga::elementTypeForInst(inst);
            if (!type) continue;
            require(inst.tile.peer,"unplaced admitted cell: "+inst.makeName());
            require(inst.coord==inst.tile->coord,"coordinate/Tile mismatch");
            require(inst.coord.x>=0 && inst.coord.x<side && inst.coord.y>=0 && inst.coord.y<side,
                    "out-of-bounds placed cell");
            // Positions encode site/column/BEL, not a raw resource bitmap bit.
            const bool reg=*type==fpga::ELEMENT_FD;
            const int lane=reg ? inst.pos/128*8 + (inst.pos%128>=64 ? 4:0) + inst.pos%16/4
                               : inst.pos/128*4 + inst.pos%16/4;
            require(inst.pos>=0 && lane>=0 && lane<16 && inst.pos%4==(reg ? 0:3),"invalid element position");
            require(occupied.emplace(inst.coord.x,inst.coord.y,*type,lane).second,"element collision");
            inst.tile->freeElementCount(*type); // Refresh lazily invalidated occupancy.
            require((inst.tile->elements_pos[*type] & (1u<<lane))!=0,"element does not exist");
            require((inst.tile->elements_free[*type] & (1u<<lane))==0,"placed element marked free");
        }
        for (const auto& [inst,where]:fixed)
            require(inst->tile.peer && inst->coord==where.first && inst->pos==where.second,
                    "fixed/completed cell moved: "+inst->makeName());
    }

    void stage(const std::string& name) {
        Json::Value row; row["stage"]=name;
        row["seconds"]=std::chrono::duration<double>(Clock::now()-last_stage).count();
        last_stage=Clock::now();
        size_t active=0;
        if (name=="outline") {
            require(++outlined==1,"Outline repeated per chunk");
            size_t estimated_combs=0;
            for (const auto& bunch:tech.estimate.data_outs) estimated_combs+=bunch.size_comb;
            require(estimated_combs==metrics["luts"].asUInt64(),"Estimate double-counted or missed shared logic");
            for (auto* inst:core) {
                require(!inst->pnr_pending,"Outline did not see whole design");
                require(inst->bunch_ref.peer,"Estimate missed core cell: "+inst->makeName());
                require(std::isfinite(inst->outline.x) && std::isfinite(inst->outline.y) &&
                        inst->outline.x>=0 && inst->outline.x<=10 && inst->outline.y>=0 && inst->outline.y<=10,
                        "invalid Outline coordinate: "+inst->makeName()+" x="+std::to_string(inst->outline.x)
                        +" y="+std::to_string(inst->outline.y)+" bunch=("+std::to_string(inst->bunch_ref->x)
                        +","+std::to_string(inst->bunch_ref->y)+")");
                require(!inst->tile.peer,"Outline prematurely assigned a core resource");
            }
            std::vector<int> lut_demand(side*side);
            for (auto* inst:core) if (inst->cell_ref->type.starts_with("LUT") && inst->cell_ref->type!="LUT1") {
                const int x=std::clamp(int(inst->outline.x*side/10),0,side-1);
                const int y=std::clamp(int(inst->outline.y*side/10),0,side-1);
                ++lut_demand[y*side+x];
            }
            int excess=0;
            for (int demand:lut_demand) excess+=std::max(0,demand-16);
            row["outline_peak_luts_per_tile"]=*std::max_element(lut_demand.begin(),lut_demand.end());
            row["outline_excess_luts"]=excess;
            row["cells"]=Json::UInt64(core.size());
        } else {
            legality();
            for (auto* inst:core) active+=!inst->pnr_pending;
            if(name=="packing") ++packed;
            if(name=="sorting") { ++sorted; require(sorted==packed,"Sorting order mismatch"); }
            if(name=="swapping") { ++swapped; require(swapped==sorted,"Swapping order mismatch"); }
            if(name=="post_swap_sorting") {
                const auto& stages=metrics["stages"];
                require(swapped==packed && !stages.empty() &&
                        stages[stages.size()-1]["stage"]=="swapping",
                        "post-swap Sorting order mismatch");
            }
            auto timing=tech.place.place_timing.analyze(tech.timings);
            require(timing.endpoints==expected_endpoints,"timing endpoints lost at "+name);
            require(std::isfinite(timing.worst_slack_ns) && std::isfinite(timing.total_negative_slack_ns),"non-finite timing");
            row["cells"]=Json::UInt64(active); row["wns_ns"]=timing.worst_slack_ns;
            row["tns_ns"]=timing.total_negative_slack_ns;
            row["violations"]=Json::UInt64(timing.violated_endpoints);
            row["unplaced_edges"]=Json::UInt64(timing.unplaced_edges);
            row["endpoints"]=Json::UInt64(timing.endpoints);
            row["run"]=Json::UInt64(packed);
        }
        metrics["stages"].append(row);
        std::cout << "\nSYNTH_STAGE " << Json::writeString(Json::StreamWriterBuilder{},row) << std::endl;
    }

    void run() {
        pnr::Pnr pnr(tech);
        pnr.run_completed=[&](const pnr::Pnr::Run& run) {
            for (auto* inst:run.cells) fixed[inst]={inst->coord,inst->pos};
        };
        const bool success = pnr.run(true);
        require(success,"placement run failed: "+pnr.failure);
        legality();
        require(outlined==1 && packed==pnr.runs.size() && sorted==packed && swapped==packed,
                "missing placement stages");
        auto final=tech.place.place_timing.analyze(tech.timings);
        require(final.endpoints==expected_endpoints && final.unplaced_edges==0,"incomplete final timing graph");
        metrics["runs"]=Json::UInt64(pnr.runs.size());
        metrics["final_wns_ns"]=final.worst_slack_ns;
        metrics["final_tns_ns"]=final.total_negative_slack_ns;
        metrics["final_violations"]=Json::UInt64(final.violated_endpoints);
        if (final.worst_slack_ns < -0.10) {
            std::sort(final.endpoint_details.begin(),final.endpoint_details.end(),
                      [](const auto& a,const auto& b) { return a.slack_ns<b.slack_ns; });
            for (size_t i=0;i<std::min(size_t{3},final.endpoint_details.size());++i) {
                const auto& endpoint=final.endpoint_details[i];
                std::cout<<"\nFAILING_PATH slack="<<endpoint.slack_ns<<" arrival="<<endpoint.arrival_ns<<'\n';
                for (const auto& edge:endpoint.critical_edges)
                    std::cout<<edge.driver->makeName()<<" ("<<edge.driver->coord.x<<','<<edge.driver->coord.y
                             <<") -> "<<edge.sink->makeName()<<" ("<<edge.sink->coord.x<<','<<edge.sink->coord.y
                             <<") wire_ns="<<edge.wire_delay_ns<<'\n';
            }
        }
        require(final.worst_slack_ns>=-0.10,"setup slack below -0.10 ns: "+std::to_string(final.worst_slack_ns));
        metrics["passed"]=true;
    }
};
}

int main(int argc, char** argv) {
    try {
        require(argc==4,"usage: placing_synth_test netlist.json period_ns metrics.json");
        Json::Value json; std::ifstream input(argv[1]); require(bool(input),"cannot open netlist"); input>>json;
        require(json["modules"].size()==1 && json["modules"].isMember("top"),"expected flattened top");
        double period=std::stod(argv[2]); require(std::isfinite(period) && period>0,"invalid clock period");
        Fixture fixture(json["modules"]["top"],period);
        try { fixture.run(); }
        catch (...) { std::ofstream(argv[3])<<fixture.metrics; throw; }
        std::ofstream(argv[3])<<fixture.metrics;
        std::cout<<"\nSynthesized placement passed\n";
    } catch (const std::exception& error) {
        std::cerr<<"\nplacing_synth_test: "<<error.what()<<'\n'; return 1;
    }
}
