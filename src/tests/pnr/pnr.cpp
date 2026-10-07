#include "pnr.h"
#include "Tech.h"
#include "Device.h"
#include "Wire.h"
#include "RouteVCC.h"
#include "RouteClocks.h"

#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
NodeMask bit(int node) { return NodeMask{0, 1} << node; }

// One register per Tile makes capacity pressure unavoidable, even in a tiny
// design. Numerical, four-direction crossbars exercise the real router.
void mesh(int width=6, int height=6) {
    auto& device = fpga::Device::current();
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.size_width=width; device.size_height=height;
    device.grid_spec.size={width,height};
    auto& type = device.tile_types.emplace_back();
    type.name="RUN_LOGIC"; type.type=fpga::Tile::TILE_LUTS;
    fpga::Element element{};
    element.name="REG0"; element.type=fpga::ELEMENT_FD;
    element.elements_to_left=fpga::ELEMENT_FD;
    type.elements.push_back(element);
    auto pin = [&](fpga::TilePinNameType direction, int resource, int local, const char* name) {
        type.pin_map.rememberResourcePinName(direction,resource,name);
        if (direction==fpga::TILE_PIN_INPUT) type.pin_map.input_nodes[resource]=bit(local);
        else type.pin_map.output_nodes[resource]=bit(local);
        type.pin_map.rememberLocalNames(direction,local,"LOCAL_"+std::to_string(local),"REG0",name);
        type.pin_map.rememberEndpointNames(direction,resource,local,"LOCAL_"+std::to_string(local),"REG0",name);
        type.pin_map.rememberEndpointRouteRef(direction,resource,local,"RUN_BOX");
    };
    pin(fpga::TILE_PIN_OUTPUT,1,0,"AQ");
    pin(fpga::TILE_PIN_INPUT,31,1,"AX");
    auto& cb=device.cb_types.emplace_back();
    cb.name="RUN_BOX"; cb.type_id=0;
    cb.rememberNodeName(fpga::CB_NODE_LOCAL,0,"LOCAL_0");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL,1,"LOCAL_1");
    std::array<fpga::Coord,4> dirs{{{0,-1},{1,0},{0,1},{-1,0}}};
    NodeMask sources{};
    for (int i=0;i<32;++i) sources|=bit(i);
    cb.local_src[0].jump=sources;
    cb.local_output_nodes=bit(0); cb.local_input_nodes=bit(1);
    for (int i=0;i<32;++i) {
        cb.rememberNodeName(fpga::CB_NODE_SRC,i,"SRC_"+std::to_string(i));
        cb.rememberNodeName(fpga::CB_NODE_DST,i,"DST_"+std::to_string(i));
        cb.dst_src[i].jump=sources;
        cb.dst_local[i].local=bit(1);
        fpga::CBType::ResolvedJump jump{};
        jump.delta=dirs[i/8]; jump.target_cb_type_id=0; jump.dsts.jump=bit(i);
        cb.dst_by_src[i].push_back(jump);
    }
    auto arc = [&](fpga::CBNodeNameType from_type,int from,fpga::CBNodeNameType to_type,int to) {
        cb.rememberConnName(from_type,from,to_type,to,*cb.nodeName(from_type,from),*cb.nodeName(to_type,to));
    };
    for(int i=0;i<32;++i) {
        arc(fpga::CB_NODE_LOCAL,0,fpga::CB_NODE_SRC,i);
        arc(fpga::CB_NODE_SRC,i,fpga::CB_NODE_DST,i);
        arc(fpga::CB_NODE_DST,i,fpga::CB_NODE_LOCAL,1);
        for(int j=0;j<32;++j) arc(fpga::CB_NODE_DST,i,fpga::CB_NODE_SRC,j);
    }
    cb.rebuildOutgoingSrcs(); cb.ensureDerivedMasks();
    device.tile_grid.resize(width*height);
    for (int y=0;y<height;++y) for(int x=0;x<width;++x) {
        auto& tile=device.tile_grid[y*width+x];
        tile.coord={x,y}; tile.cb_coord=tile.coord; tile.name=tile.coord;
        tile.type=fpga::Tile::TILE_LUTS; tile.tile_type=&type;
        tile.cb_type=tile.cb.type=&cb;
        tile.full_name="TILE_"+std::to_string(x)+"_"+std::to_string(y);
    }
    device.rebuildIncomingDstMasks();
}

struct Fixture {
    technology::Tech tech;
    Referable<rtl::Module>* parent;
    Referable<rtl::Module>* primitive;
    std::list<Referable<rtl::Cell>> cells;
    std::vector<rtl::Inst*> insts;
    Fixture(size_t count, int side=6) {
        mesh(side,side); tech.init();
        tech.design.GND=nullptr; tech.design.VCC=nullptr;
        tech.place.write_debug_images=false;
        tech.sorting.config.maximum_runtime_seconds=0.1;
        tech.swapping.config.maximum_runtime_seconds=0.1;
        parent=&tech.design.modules.emplace_back(); parent->name="run_top";
        parent->is_blackbox=false; parent->nets.reserve(count);
        primitive=&tech.design.modules.emplace_back(); primitive->name="run_primitive";
        primitive->is_blackbox=true; primitive->parent_ref.set(parent);
        tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
        tech.design.top_cell.module_ref.set(parent);
        tech.design.top.cell_ref.set(&tech.design.top_cell); tech.design.top.depth=0;
        for(size_t i=0;i<count;++i) {
            auto& cell=cells.emplace_back(); cell.name="cell_"+std::to_string(i);
            cell.type="FD"; cell.module_ref.set(primitive);
            cell.ports.reserve(3);
            rtl::Port input{}; input.name="D"; input.type=rtl::Port::PORT_IN;
            rtl::Port output{}; output.name="Q"; output.type=rtl::Port::PORT_OUT;
            cell.ports.emplace_back(std::move(input)); cell.ports.emplace_back(std::move(output));
            auto& inst=tech.design.top.insts.emplace_back();
            inst.cell_ref.set(&cell); inst.parent_ref.set(&tech.design.top);
            inst.depth=1; inst.cnt_inputs=1; inst.cnt_outputs=1; inst.cnt_clocks=1;
            inst.pos=-1; inst.coord={-1,-1}; inst.outline={5,5,false};
            inst.stats.max_deficit=double(count-i);
            inst.conns.reserve(3);
            for(auto& port:cell.ports) {
                auto& conn=inst.conns.emplace_back(); conn.port_ref.set(&port); conn.inst_ref.set(&inst);
            }
            auto& bunch=tech.estimate.data_outs.emplace_back(pnr::RegBunch{});
            bunch.reg=&inst; bunch.size=1; bunch.size_regs=bunch.size_regs_own=1;
            bunch.x=bunch.y=5; inst.bunch_ref.set(&bunch);
            insts.push_back(&inst);
        }
    }
    rtl::Net& connect(size_t from,size_t to) {
        auto& output=insts[from]->conns[1]; auto& input=insts[to]->conns[0];
        int id=output.port_ref->designator;
        if(id<=0) {
            id=parent->nets.size()+1;
            auto& net=parent->nets.emplace_back(); net.name="net_"+std::to_string(id);
            net.designators.push_back(id); output.port_ref->designator=id;
        }
        input.port_ref->designator=id; input.set(&output);
        return parent->nets[id-1];
    }
    void clock() {
        auto& device=fpga::Device::current();
        auto& type=device.tile_types.front(); auto& cb=device.cb_types.front();
        fpga::SiteModel site;
        site.name="CLOCK_SITE"; site.type="CLOCK_BUFFER"; site.pos=0;
        for (auto [name,node] : {std::pair{"O",2}, {"CLK",3}}) {
            auto direction=node==2?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT;
            auto& pin=site.pins.emplace_back(); pin.port=name; pin.site_pos=0;
            pin.direction=node==2?fpga::Pin::PIN_OUTPUT:fpga::Pin::PIN_INPUT;
            type.pin_map.rememberResourcePinName(direction,node,name);
            (node==2?type.pin_map.output_nodes:type.pin_map.input_nodes)[node]=bit(node);
            cb.rememberNodeName(fpga::CB_NODE_LOCAL,node,"CLOCK_"+std::to_string(node));
        }
        type.sites.push_back(site);
        for(int y=-5;y<=5;++y) for(int x=-5;x<=5;++x)
            cb.local_by_local[2].push_back({{x,y},0,fpga::CB_NODE_LOCAL,bit(3)});
        auto& cell=cells.emplace_back(); cell.name="clock_source"; cell.type="CLOCK_BUFFER";
        cell.module_ref.set(primitive);
        auto& port=cell.ports.emplace_back(); port.name="O"; port.type=rtl::Port::PORT_OUT;
        port.designator=parent->nets.size()+1;
        auto& net=parent->nets.emplace_back(); net.name="clock_net"; net.designators.push_back(port.designator);
        auto& source=tech.design.top.insts.emplace_back(); source.cell_ref.set(&cell);
        source.parent_ref.set(&tech.design.top); source.depth=1; source.pos=0;
        source.coord={5,5}; source.outline={5/0.6F,5/0.6F,true};
        device.tile_grid.back().assign(&source);
        auto& output=source.conns.emplace_back(); output.port_ref.set(&port); output.inst_ref.set(&source);
        tech.buffers_ports.emplace("CLOCK_BUFFER","O");
        for(auto* inst:insts) {
            auto& pin=inst->cell_ref->ports.emplace_back(); pin.name="C";
            pin.type=rtl::Port::PORT_IN; pin.designator=port.designator; pin.index=1;
            auto& input=inst->conns.emplace_back(); input.port_ref.set(&pin);
            input.inst_ref.set(static_cast<Referable<rtl::Inst>*>(inst)); input.set(&output);
            inst->cnt_inputs=2;
        }
        tech.clocks.clocks_list.emplace_back(rtl::Clock{
            .name="run_clock", .conn_ptr=&output, .conn_name="clock_source.O", .period_ns=10, .duty=50});
        tech.prepareTimingLists();
    }
};

void planner() {
    Fixture f(9);
    // Shuffled rank, stable ties, an indivisible bunch and an unestimated cell.
    f.insts[7]->stats.max_deficit=100;
    f.insts[1]->bunch_ref.set(f.insts[7]->bunch_ref.peer);
    f.insts[8]->bunch_ref.clear();
    pnr::Pnr flow(f.tech); flow.run_cells=3; flow.plan();
    require(flow.runs.front().cells.size()==3,"wrong first run size");
    require(std::find(flow.runs.front().cells.begin(),flow.runs.front().cells.end(),f.insts[7])!=flow.runs.front().cells.end(),"critical group not first");
    require(std::find(flow.runs.front().cells.begin(),flow.runs.front().cells.end(),f.insts[1])!=flow.runs.front().cells.end(),"bunch split");
    std::unordered_set<rtl::Inst*> seen;
    for(auto& run:flow.runs) for(auto* cell:run.cells) {
        require(seen.insert(cell).second,"duplicate cell");
        require(!cell->pnr_pending&&!cell->pnr_completed&&!cell->tile.peer,"plan mutated physical state");
    }
    require(seen.size()==9,"planner lost orphan cell");
    flow.run_cells=1; flow.plan();
    require(flow.runs.front().cells.size()==2,"oversized bunch was split");
    flow.run_cells=0;
    bool rejected=false; try {flow.plan();} catch(const std::invalid_argument&) {rejected=true;}
    require(rejected,"zero target accepted");
    Fixture chains(5);
    chains.insts[0]->cell_ref->type="LUT6";
    chains.insts[1]->cell_ref->type="MUXF7";
    chains.insts[2]->cell_ref->type="MUXF8";
    chains.insts[3]->cell_ref->type="CARRY4";
    chains.insts[4]->cell_ref->type="CARRY4";
    chains.connect(0,1); chains.connect(1,2); chains.connect(3,4);
    pnr::Pnr chain_plan(chains.tech); chain_plan.run_cells=1; chain_plan.plan();
    require(chain_plan.runs.size()==2 && chain_plan.runs[0].cells.size()==3 &&
            chain_plan.runs[1].cells.size()==2,"strict mux/carry chain split across runs");
    Fixture domains(6);
    domains.insts[0]->cell_ref->type="CLOCK_BUFFER";
    domains.insts[1]->cell_ref->type="CLOCK_BUFFER";
    for(size_t i=2;i<6;++i) domains.insts[i]->conns[0].port_ref->name="C";
    domains.connect(0,2); domains.connect(0,3);
    domains.connect(1,4); domains.connect(1,5);
    pnr::Pnr domain_plan(domains.tech); domain_plan.run_cells=2; domain_plan.plan();
    require(domain_plan.runs.size()==3,"clock domain collapsed into one indivisible group");
    for(auto& run:domain_plan.runs) for(size_t source=0;source<2;++source)
        if(std::find(run.cells.begin(),run.cells.end(),domains.insts[source])!=run.cells.end())
            require(std::find(run.cells.begin(),run.cells.end(),domains.insts[source?4:2])!=run.cells.end(),
                    "clock source admitted without its first consumer");
}

// Extending a frozen clock tree must search once for the new batch, not
// rediscover the same graph separately for every admitted register.
void incrementalClockBatch(size_t count=65) {
    Fixture f(count,10);
    f.clock();
    auto& device=fpga::Device::current();
    auto& cb=device.cb_types.front();
    cb.local_by_local[2].clear();
    for(int y=-9;y<=9;++y) for(int x=-9;x<=9;++x)
        cb.local_by_local[2].push_back({{x,y},0,fpga::CB_NODE_LOCAL,bit(3)});
    for(size_t i=0;i<f.insts.size();++i) {
        auto& inst=*f.insts[i];
        inst.pos=0; inst.coord=device.tile_grid[i].coord;
        device.tile_grid[i].assign(&inst);
        inst.pnr_pending=i!=0;
    }
    pnr::RouteClocks router(f.tech,device);
    require(router.routeDesign(f.tech.clocks),"initial clock failed");
    require(router.stats().routed==1,"initial clock did not route one sink");
    auto& net=f.parent->nets.back();
    require(net.routes.size()==1,"initial binding count");
    const auto old_id=net.routes.front().route_id;
    const auto old_wires=f.insts.front()->wires;
    f.insts.front()->pnr_completed=true;
    for(auto* inst:f.insts) inst->pnr_pending=false;
    require(router.routeDesign(f.tech.clocks),"retained clock extension failed");
    require(router.stats().routed==count-1,"extension skipped sinks");
    require(net.routes.size()==count&&net.routes.front().route_id==old_id,
            "clock extension replaced or duplicated retained binding");
    const auto& wires=f.insts.front()->wires;
    require(wires.size()==old_wires.size()&&wires.front().size()==old_wires.front().size(),
            "retained clock path changed");
    for(size_t i=0;i<wires.front().size();++i) {
        const auto& a=wires.front()[i]; const auto& b=old_wires.front()[i];
        require(a.type==b.type&&a.from.x==b.from.x&&a.from.y==b.from.y&&
                a.to.x==b.to.x&&a.to.y==b.to.y&&a.local==b.local&&a.jump==b.jump,
                "retained clock wire changed");
    }
    std::cout<<"incremental clock batch visited="<<router.stats().graph_nodes<<'\n';
    require(router.stats().graph_nodes<=101,
            "retained clock graph was searched repeatedly for the new sinks: "+
            std::to_string(router.stats().graph_nodes));
}

// Exercise both the small-tree and batched clock routers: after sink 1 fails,
// no later (physically reachable) sink may acquire a route.
void clockFailureStopsSinks() {
    for (size_t count : {size_t{5}, size_t{40}}) {
        Fixture f(count,10);
        f.clock();
        auto& device=fpga::Device::current();
        auto& cb=device.cb_types.front();
        cb.local_by_local[2].clear();
        for(int y=-9;y<=9;++y) for(int x=-9;x<=9;++x)
            cb.local_by_local[2].push_back({{x,y},0,fpga::CB_NODE_LOCAL,bit(3)});
        for(size_t i=0;i<f.insts.size();++i) {
            f.insts[i]->pos=0;
            f.insts[i]->coord=device.tile_grid[i].coord;
            device.tile_grid[i].assign(f.insts[i]);
        }
        device.tile_grid[1].cb.local.local |= bit(3);
        device.tile_grid[1].pin_state.lease(3);
        pnr::RouteClocks router(f.tech,device);
        require(!router.routeDesign(f.tech.clocks),"blocked clock sink did not fail");
        require(router.stats().routed==1 && router.stats().failed==1,
                "clock router continued after the first failed sink");
        require(!f.insts[0]->wires.empty(),"clock router lost its completed first sink");
        for(size_t i=1;i<count;++i)
            require(f.insts[i]->wires.empty(),"clock router routed beyond the failed sink");
    }
}

void clockRepairCleanup() {
    Fixture f(2);
    f.clock();
    auto& device=fpga::Device::current();
    for(size_t i=0;i<f.insts.size();++i) {
        f.insts[i]->pos=0; f.insts[i]->coord=device.tile_grid[i].coord;
        device.tile_grid[i].assign(f.insts[i]);
    }
    pnr::RouteClocks router(f.tech,device);
    require(router.routeDesign(f.tech.clocks),"initial clock failed");
    auto& net=f.parent->nets.back();
    auto& root=device.tile_grid.back();
    require(fpga::unrouteNetRoute(net,0),"first clock branch not removed");
    require((root.cb.local.local&bit(2))!=NodeMask{},"surviving clock root lost lease");
    require(fpga::unrouteNetRoute(net,1),"last clock branch not removed");
    require((root.cb.local.local&bit(2))==NodeMask{},
            "removed clock tree left its root leased, preventing repair");
    require((root.pin_state.leased_nodes&bit(2))==NodeMask{},"clock root pin lease leaked");
    require(router.routeDesign(f.tech.clocks),"clock tree cannot be rebuilt after cleanup");
    require(router.stats().routed==2,"repaired clock sinks missing");
}

void incremental(bool with_clock=false) {
    Fixture f(8);
    f.connect(0,1); f.connect(0,3); f.connect(0,6);
    f.connect(2,4); f.connect(5,2); // Future driver -> already admitted sink.
    f.connect(6,7);
    if (with_clock) f.clock();
    pnr::Pnr flow(f.tech); flow.run_cells=3;
    struct Frozen {rtl::Inst* cell; int x,y,pos;};
    std::vector<Frozen> frozen;
    struct SavedRoute {rtl::Net* net; uint64_t id; std::vector<fpga::Wire> wires;};
    std::vector<SavedRoute> saved;
    size_t callbacks=0;
    flow.run_completed=[&](const pnr::Pnr::Run& run) {
        for (const auto& old : saved) {
            auto found=std::find_if(old.net->routes.begin(),old.net->routes.end(),
                [&](const auto& b){return b.route_id==old.id;});
            require(found!=old.net->routes.end(),"completed route binding disappeared");
            const auto& wires=found->owner->wires.at(found->route_index);
            require(wires.size()==old.wires.size(),"completed route length changed");
            for(size_t i=0;i<wires.size();++i) {
                const auto& a=wires[i]; const auto& b=old.wires[i];
                require(a.type==b.type&&a.from.x==b.from.x&&a.from.y==b.from.y&&
                    a.to.x==b.to.x&&a.to.y==b.to.y&&a.local==b.local&&a.jump==b.jump&&
                    a.dst==b.dst&&a.pos==b.pos&&a.shared==b.shared&&a.owns_dst==b.owns_dst,
                    "completed route was rewritten");
            }
        }
        for(auto& old:frozen)
            require(old.cell->coord.x==old.x&&old.cell->coord.y==old.y&&old.cell->pos==old.pos,
                    "completed placement changed in a later run");
        for(auto* cell:run.cells) {
            require(cell->pnr_completed&&cell->outline.fixed&&cell->tile.peer,"cell not frozen");
            frozen.push_back({cell,cell->coord.x,cell->coord.y,cell->pos});
        }
        for(auto* cell:f.insts) if(cell->pnr_pending) require(!cell->tile.peer,"pending cell got placed");
        for(auto& net:f.parent->nets) for(auto& binding:net.routes)
            require(binding.owner&&fpga::isRouteComplete(binding.owner->wires.at(binding.route_index)),"incomplete admitted route");
        saved.clear();
        std::vector<rtl::Net*> nets;
        for(auto& net:f.parent->nets) nets.push_back(&net);
        for(auto& tile:fpga::Device::current().tile_grid) {
            std::ostringstream proof;
            auto audit=fpga::auditTileCongestion(tile,proof,nets);
            require(!audit.orphan_leases&&!audit.missing_leases&&
                    !audit.missing_registrations&&!audit.stale_registrations,
                    "cross-run resource ownership mismatch: "+proof.str());
        }
        for(auto& net:f.parent->nets) for(auto& b:net.routes)
            saved.push_back({&net,b.route_id,b.owner->wires.at(b.route_index)});
        ++callbacks;
    };
    bool success=flow.run();
    require(success,"incremental flow failed: "+flow.failure);
    require(callbacks==3,"not all runs completed");
    require(flow.runs.front().deferred_connections>0,"no boundary edges reported");
    require(flow.runs.back().deferred_connections==0,"boundary edges remain at end");
    size_t routes=0;
    for(auto& net:f.parent->nets) {
        require(net.pnr_preserved&&!net.routeCanBePreempted(),"completed net can be preempted");
        routes+=net.routes.size();
    }
    require(routes==(with_clock?14:6),"lost or duplicated connection across runs");
}

void failureStopsAdmission() {
    Fixture f(3); mesh(1,1);
    pnr::Pnr flow(f.tech); flow.run_cells=1;
    require(!flow.run(),"over-capacity flow unexpectedly succeeded");
    require(flow.runs[0].success&&!flow.runs[1].success&&!flow.runs[2].success,
            "wrong run failed");
    require(f.insts[0]->pnr_completed&&f.insts[0]->outline.fixed,"lost successful first run");
    require(f.insts[2]->pnr_pending&&!f.insts[2]->tile.peer,"admitted work after failure");
    require(!f.tech.route.preserve_routes,"incremental mode leaked after failure");
}

void constants() {
    Fixture f(2);
    Referable<rtl::Port> port;
    port.name="one"; port.type=rtl::Port::PORT_OUT; port.is_global=true;
    Referable<rtl::Conn> one;
    one.port_ref.set(&port); one.inst_ref.set(&f.tech.design.top);
    f.tech.design.VCC=&one;
    for(auto* cell:f.insts) cell->conns[0].set(&one);
    auto& cb=fpga::Device::current().cb_types.front();
    cb.constant_one_nodes=bit(4);
    cb.rememberNodeName(fpga::CB_NODE_LOCAL,4,"ONE");
    cb.local_local[4].local=bit(1);
    cb.ensureDerivedMasks();
    pnr::Pnr flow(f.tech); flow.run_cells=1;
    bool success=flow.run();
    require(success,"incremental constant flow failed: "+flow.failure);
    size_t branches=0;
    for(auto& net:f.parent->nets) if(net.distributed_source) {
        require(net.pnr_preserved,"constant route not preserved");
        for(auto& binding:net.routes) {
            require(fpga::isRouteComplete(binding.owner->wires.at(binding.route_index)),"lost constant branch");
            ++branches;
        }
    }
    require(branches==2,"constant sinks lost or rerouted across runs");
}

void largerMesh(bool branching=false) {
    Fixture f(240,20);
    for(size_t i=1;i<f.insts.size();++i) f.connect(branching?(i-1)/3:i-1,i);
    pnr::Pnr flow(f.tech); flow.run_cells=30;
    rtl::Net* storage=nullptr;
    flow.run_completed=[&](const auto&) {
        if (storage) require(storage==&f.parent->nets.front(),"retained Net addresses changed");
        storage=&f.parent->nets.front();
    };
    bool success=flow.run();
    require(success,"240-cell mesh failed: "+flow.failure);
    require(flow.runs.size()==8&&flow.runs.back().completed_cells==240,"scale coverage incorrect");
    size_t routes=0;
    for(auto& net:f.parent->nets) routes+=net.routes.size();
    require(routes==239,"scale run lost connections");
}

void movingWithDeferredInput() {
    // The current source must relocate, but its data driver belongs to a
    // later run. Prove/route only admitted inputs; the future edge must stay
    // deferred through both candidate probing and relocation commit.
    Fixture f(3,3);
    f.connect(0,1);
    auto& output=f.connect(1,2);
    f.insts[0]->pnr_pending=true;
    auto& device=fpga::Device::current();
    fpga::CBType isolated=device.cb_types.front();
    isolated.local_src[0].jump={};
    isolated.rebuildOutgoingSrcs();
    auto* old=device.getTile(0,0);
    old->cb_type=old->cb.type=&isolated;
    for(auto [cell,tile]:{std::pair{f.insts[1],old},
                        std::pair{f.insts[2],device.getTile(2,2)}}) {
        cell->pos=0;
        tile->assign(cell);
        cell->coord=tile->coord;
    }
    auto& router=f.tech.route;
    router.fpga=&device;
    router.fpga_width=router.fpga_height=3;
    router.aspect_x=router.aspect_y=0.3F;
    router.preserve_routes=true;
    pnr::RouteDesign::RouteTask task{
        f.insts[1],f.insts[2],&output,"Q","D","current_run_edge"};
    std::vector<pnr::RouteDesign::RouteTask> tasks;
    std::string reason;
    bool moved=false;
    f.insts[0]->pnr_pending=false;
    require(!router.moveUnfinishedSource(task,&tasks,&reason,nullptr,&moved)
                && reason.find("invalid endpoint")!=std::string::npos,
            "an admitted unplaced driver must still reject relocation: "+reason);
    require(f.insts[1]->tile.peer==old && output.routes.empty(),
            "rejected input proof mutated the source");
    f.insts[0]->pnr_pending=true;
    bool success=router.moveUnfinishedSource(task,&tasks,&reason,nullptr,&moved);
    require(success,"source relocation included future input: "+reason);
    require(moved && f.insts[1]->tile.peer!=old,"fixture did not force relocation");
    require(!f.insts[0]->tile.peer && f.parent->nets.front().routes.empty(),
            "relocation admitted a future driver or routed its boundary edge");
    require(!output.routes.empty(),"relocated source lost current output");
    for(const auto& binding:output.routes)
        require(fpga::isRouteComplete(binding.owner->wires.at(binding.route_index)),
                "source relocation left current output incomplete");
    for(const auto& queued:tasks)
        require(!queued.from->pnr_pending && !queued.to->pnr_pending,
                "relocation queued deferred connection");
}

void perRunBudgets() {
    // Only the first register is admitted initially. Its upstream chain is
    // future work, not 240 cells worth of Outline/routing work in this run.
    Fixture f(240,20);
    for(size_t i=1;i<f.insts.size();++i) {
        f.connect(i,i-1);
        f.insts[i]->bunch_ref.set(f.insts[1]->bunch_ref.peer);
    }
    pnr::Pnr flow(f.tech); flow.run_cells=1;
    size_t callbacks=0;
    flow.run_completed=[&](const auto&) {
        if(callbacks++==0) {
            require(f.tech.outline.iteration_limit==1,
                    "Outline budget includes future-run cells: "+
                    std::to_string(f.tech.outline.iteration_limit));
            require(f.tech.route.iteration_limit==16,
                    "routing budget includes future-run cells: "+
                    std::to_string(f.tech.route.iteration_limit));
        } else {
            require(f.tech.outline.iteration_limit==23 && f.tech.route.iteration_limit==23,
                    "second run was not sized from its 239 admitted cells");
        }
    };
    bool success=flow.run();
    require(success,"per-run budget regression failed: "+flow.failure);
    require(callbacks==2,"budget test did not exercise both runs");
}
}
int main(int argc,char** argv) {
    try {
        if(argc==1||std::string(argv[1])=="planner") planner();
        if(argc==1||std::string(argv[1])=="incremental") incremental();
        if(argc==1||std::string(argv[1])=="clock") incremental(true);
        if(argc==1||std::string(argv[1])=="clock_batch") {
            incrementalClockBatch(30); // below the fresh-tree batch threshold
            incrementalClockBatch();
        }
        if(argc==1||std::string(argv[1])=="clock_cleanup") clockRepairCleanup();
        if(argc==1||std::string(argv[1])=="clock_fail_fast") clockFailureStopsSinks();
        if(argc==1||std::string(argv[1])=="failure") failureStopsAdmission();
        if(argc==1||std::string(argv[1])=="constants") constants();
        if(argc==1||std::string(argv[1])=="mesh") largerMesh();
        if(argc>1&&std::string(argv[1])=="mesh_fanout") largerMesh(true);
        if(argc>1&&std::string(argv[1])=="moving_deferred") movingWithDeferredInput();
        if(argc>1&&std::string(argv[1])=="run_budget") perRunBudgets();
        std::cout<<"\nPnR run regressions passed\n"; return 0;
    } catch(const std::exception& e) {std::cerr<<"\nFAIL: "<<e.what()<<'\n'; return 1;}
}
