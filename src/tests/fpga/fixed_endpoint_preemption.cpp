#include "Tech.h"
#include "Wire.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <initializer_list>
#include <limits>
#include <list>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

namespace {
void require(bool ok, const std::string &message) {
  if (!ok) throw std::runtime_error(message);
}

// Captured Example 1 wire-only recovery had this short complete approach:
// LOCAL -> SRC -> DST -> SRC -> DST -> SRC66 -> DST66 -> LOCAL.
// Only the last jump's two transit resources needed preemption. Fanouts
// accepted the preview, but Moving rejected it because its wire-only call
// had no selected focus. Preserve that three-jump geometry here, translated
// into a 6x5 abstract device, with real owners and a retained earlier gap.
// A protected-Takeoff control occupies the same exit and must remain uncut.
struct Fixture {
  technology::Tech tech;
  fpga::Device &device = fpga::Device::current();
  Referable<rtl::Module> *parent, *primitive;
  std::list<Referable<rtl::Cell>> cells;
  std::list<Referable<pnr::RegBunch>> bunches;
  rtl::Inst *source, *owner, *stable, *request_source, *request_sink, *foreign_source, *foreign_sink;
  rtl::Net *signal, *request=nullptr, *foreign;

  explicit Fixture(bool protected_takeoff=false, bool extended=false) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=6; device.size_height=5; device.grid_spec.size={6,5};
    device.tile_grid.resize(30); device.tile_types.reserve(30); device.cb_types.reserve(30);
    for(int y=0;y<5;++y) for(int x=0;x<6;++x) {
      auto &t=*device.getTile(x,y);
      auto &cb=device.cb_types.emplace_back();
      cb.type_id=cb.base_type_id=y*6+x;
      cb.name="matrix_"+std::to_string(1701+y*6+x);
      auto &type=device.tile_types.emplace_back();
      type.name="resource_"+std::to_string(1901+y*6+x);
      type.type=fpga::Tile::TILE_LUTS;
      type.pin_map.output_nodes[16].setBit(154);
      type.pin_map.nodes[{"LUT6","I0",0}].setBit(94);
      t.coord=t.cb_coord={x,y}; t.name=t.coord;
      t.full_name="tile_"+std::to_string(x)+"_"+std::to_string(y);
      t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
      name(t,fpga::CB_NODE_LOCAL,154); name(t,fpga::CB_NODE_LOCAL,94);
      cb.local_output_nodes.setBit(154); cb.local_input_nodes.setBit(94);
    }
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    tech.route.preserve_routes=true;
    parent=&tech.design.modules.emplace_back(); parent->name="focused_wire_top";
    parent->is_blackbox=false; parent->nets.reserve(64);
    primitive=&tech.design.modules.emplace_back(); primitive->name="logic_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent);
    tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("source_401",{0,0}); owner=cell("owner_402",{4,3});
    stable=cell("stable_403",{5,4});
    request_source=cell("request_source_501",{1,0});
    request_sink=cell("request_sink_502",{0,1});
    foreign_source=cell("transit_source_601",protected_takeoff?fpga::Coord{4,1}:fpga::Coord{5,0});
    foreign_sink=cell("transit_sink_602",{0,4});
    signal=connect(*source,*owner); connect(*source,*stable);
    foreign=connect(*foreign_source,*foreign_sink);

    // The owner's old path uses JOINT33, preempted by the next Basic run.
    edge({0,0},{0,1},154,17,20,true);
    auto &cross=*device.getTile(0,1)->cb_type;
    cross.dst_joint[20].joint.setBit(33);
    cross.joint_src[33].jump.setBit(246);
    name(*device.getTile(0,1),fpga::CB_NODE_JOINT,33);
    edge({0,1},{2,1},20,246,120);
    cross.dst_src[20].jump.clearBit(246);
    edge({2,1},{4,3},120,140,220); terminal({4,3},220);
    edge({0,0},{5,4},154,102,223,true); terminal({5,4},223);

    // The captured short alternative: (0,0)->(0,1)->(4,1)->(4,3).
    if (extended) {
      // The real witness has eight jumps / eighteen nodes and four cuts.
      // Keep that path length; put the four occupied resources on two
      // consecutive transit jumps so their combined validation is necessary.
      edge({0,1},{1,1},20,258,262);
      edge({1,1},{1,2},262,287,263);
      edge({1,2},{2,2},263,288,264);
      edge({2,2},{3,2},264,289,265);
      edge({3,2},{4,1},265,290,266);
      edge({4,1},{5,2},266,66,240);
      edge({5,2},{4,3},240,284,66);
    } else {
      edge({0,1},{4,1},20,258,262);
      edge({4,1},{4,3},262,66,66);
    }
    terminal({4,3},66);

    edge({1,0},{1,1},154,111,210,true);
    edge({1,1},{0,1},210,112,201);
    cross.dst_joint[201].joint.setBit(33);
    cross.joint_local[33].local.setBit(94);

    if(protected_takeoff) {
      if (extended) edge({4,1},{5,2},154,66,240,true);
      else edge({4,1},{4,3},154,66,66,true);
    } else {
      edge({5,0},{4,1},154,110,230,true);
      if (extended) edge({4,1},{5,2},230,66,240);
      else edge({4,1},{4,3},230,66,66);
      // A longer free repair bypass for the displaced transit owner. No new
      // edges are added after generation; free-first recovery must use it.
      if (extended) {
        edge({4,1},{5,1},230,281,241);
        edge({5,1},{4,0},241,282,242);
        edge({4,0},{5,4},242,283,243);
        edge({5,4},{0,4},243,285,222);
      } else {
        edge({4,1},{5,2},230,281,240);
        edge({5,2},{5,4},240,282,241);
        edge({5,4},{0,4},241,283,222);
      }
    }
    edge({4,3},{0,4},66,334,222); terminal({0,4},222);
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
  }
  static void name(fpga::Tile &t,fpga::CBNodeNameType kind,int id) {
    t.cb_type->rememberNodeName(kind,id,"node_"+
        std::to_string((unsigned(id)*2654435761U)^(unsigned(kind)*101U)));
  }
  void edge(fpga::Coord a,fpga::Coord b,int input,int src,int dst,bool takeoff=false) {
    auto &t=*device.getTile(a.x,a.y); auto &to=*device.getTile(b.x,b.y);
    auto &cb=*t.cb_type;
    name(t,takeoff?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST,input);
    name(t,fpga::CB_NODE_SRC,src); name(to,fpga::CB_NODE_DST,dst);
    if(takeoff) cb.local_src[input].jump.setBit(src);
    else cb.dst_src[input].jump.setBit(src);
    fpga::CBType::ResolvedJump jump{};
    jump.delta=b-a; jump.target_tile_coord=true;
    jump.target_cb_type_id=to.cb_type->type_id; jump.dsts.jump.setBit(dst);
    auto &landings=cb.dst_by_src[src];
    if(landings.empty()) landings.push_back(jump);
    else require(landings.size()==1 && landings.front().delta.x==jump.delta.x &&
                 landings.front().delta.y==jump.delta.y &&
                 landings.front().target_cb_type_id==jump.target_cb_type_id &&
                 landings.front().dsts.jump==jump.dsts.jump,
                 "one physical SRC was assigned two different landings");
  }
  void terminal(fpga::Coord c,int dst) {
    device.getTile(c.x,c.y)->cb_type->dst_local[dst].local.setBit(94);
  }
  rtl::Inst *cell(const std::string &label,fpga::Coord c,int pos=0) {
    auto &type=cells.emplace_back(); type.name=label; type.type="LUT6";
    type.module_ref.set(primitive); type.ports.reserve(2);
    for(auto direction:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &p=type.ports.emplace_back(); p.type=direction;
      p.name=direction==rtl::Port::PORT_IN?"I0":"O"; p.designator=-1;
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type);
    i.parent_ref.set(&tech.design.top); i.depth=1; i.pos=pos; i.coord=c;
    i.outline.fixed=true; i.cnt_inputs=1; i.cnt_outputs=1; i.conns.reserve(2);
    for(auto &p:type.ports) {
      auto &conn=i.conns.emplace_back(); conn.port_ref.set(&p); conn.inst_ref.set(&i);
    }
    device.getTile(c.x,c.y)->assign(&i); return &i;
  }
  rtl::Net *connect(rtl::Inst &a,rtl::Inst &b) {
    auto &out=a.conns[1]; auto &in=b.conns[0]; int id=out.port_ref->designator;
    if(id<0) {
      id=parent->nets.size()+1;
      auto &net=parent->nets.emplace_back(); net.name="signal_"+std::to_string(id);
      net.designators.push_back(id); out.port_ref->designator=id;
    }
    in.port_ref->designator=id; in.set(&out); return &parent->nets[id-1];
  }
  void run(std::initializer_list<rtl::Inst *> sinks) {
    bunches.clear();
    for(auto *sink:sinks) {
      auto &b=bunches.emplace_back(); b.reg=sink; b.size=1; b.size_comb=1;
    }
    tech.route.routeDesign(bunches);
  }
  std::vector<fpga::Wire> *route(rtl::Net &net,rtl::Inst &sink) {
    for(auto &b:net.routes) if(b.to==&sink && b.owner && b.route_index<b.owner->wires.size())
      return &b.owner->wires[b.route_index];
    return nullptr;
  }
  bool complete(rtl::Net &net,rtl::Inst &sink) {
    auto *w=route(net,sink); return w && fpga::isRouteComplete(*w);
  }
  void audit() {
    for(auto &t:device.tile_grid) {
      std::ostringstream report;
      std::vector<rtl::Net *> nets{signal,foreign};
      if(request) nets.push_back(request);
      auto a=fpga::auditTileCongestion(t,report,nets);
      require(!a.orphan_leases&&!a.missing_leases&&!a.missing_registrations&&
              !a.stale_registrations&&!a.conflicting_driver_nodes,report.str());
    }
  }
};
} // namespace

namespace {
struct FanoutBudget {
  std::optional<std::string> old;
  explicit FanoutBudget(bool expire) {
    if(const char* value=std::getenv("SCALEPNR_ROUTE_FANOUT_TIMEOUT")) old=value;
    setenv("SCALEPNR_ROUTE_FANOUT_TIMEOUT",expire?"1e-99":"60",1);
  }
  ~FanoutBudget() {
    if(old) setenv("SCALEPNR_ROUTE_FANOUT_TIMEOUT",old->c_str(),1);
    else unsetenv("SCALEPNR_ROUTE_FANOUT_TIMEOUT");
  }
};

// Inspect existing phase counters instead of asserting a wall-clock runtime.
// Preserve stderr and the caller's profiling setting even if routing fails.
struct RepairProfileCapture {
  std::FILE *file=std::tmpfile();
  int saved=dup(STDERR_FILENO);
  std::optional<std::string> previous;
  RepairProfileCapture() {
    require(file && saved>=0,"cannot capture fixed-endpoint repair profile");
    if(const char *value=std::getenv("SCALEPNR_PROFILE_REPAIR")) previous=value;
    std::fflush(stderr);
    require(dup2(fileno(file),STDERR_FILENO)>=0,"cannot redirect repair profile");
    setenv("SCALEPNR_PROFILE_REPAIR","1",1);
  }
  ~RepairProfileCapture() {
    std::fflush(stderr); dup2(saved,STDERR_FILENO); close(saved); std::fclose(file);
    if(previous) setenv("SCALEPNR_PROFILE_REPAIR",previous->c_str(),1);
    else unsetenv("SCALEPNR_PROFILE_REPAIR");
  }
  std::string read() {
    std::fflush(stderr); std::rewind(file);
    std::string result; char buffer[4096];
    while(size_t n=std::fread(buffer,1,sizeof(buffer),file)) result.append(buffer,n);
    return result;
  }
};
}

void fixed_endpoint_preemption_regression(int mode) {
  debug_level=1;
  const bool movable_owner=mode>=3 && mode<=5;
  const bool protected_takeoff=mode==2 || mode==4;
  Fixture f(protected_takeoff,movable_owner);
  // Install the foreign owner first so the original owner must use its free
  // old path, rather than consuming the contested alternative during setup.
  f.run({f.foreign_sink});
  require(f.complete(*f.foreign,*f.foreign_sink),"foreign reference route did not complete");
  require(f.device.getTile(4,1)->cb.src.jump.testBit(66) &&
          f.device.getTile(4,3)->cb.dst.jump.testBit(66),
          "reference owner did not occupy both contested jump resources");
  const auto owners=fpga::findNetOwnersByNode(*f.device.getTile(4,1),fpga::CB_NODE_SRC,66);
  require(owners.size()==1 && owners.front().net==f.foreign,
          "contested SRC has no real exclusive owner");
  const std::vector<fpga::RouteCutNode> cut{{{4,1},fpga::CB_NODE_SRC,66}};
  require(fpga::canPreemptNetRouteFromNodes(*f.foreign,owners.front().binding_index,cut)
              ==!protected_takeoff,"fixture did not establish transit/Takeoff eligibility");
  f.run({f.owner,f.stable});
  require(f.complete(*f.signal,*f.owner) && f.complete(*f.signal,*f.stable),
          "original owner and stable sibling did not complete");
  require(f.device.getTile(0,1)->cb.joint.jump.testBit(33),
          "original owner did not use the primary contested JOINT");
  if (movable_owner) {
    // Example 1 tried relocating an unfixed LUT, rejected every reachable
    // host, and failed. A read-only replay at its ORIGINAL position found the
    // eighteen-node/four-cut route above. Make relocation genuinely impossible
    // here using occupied one-position Element models, not search hints.
    for (auto &inst : f.tech.design.top.insts) inst.cnt_inputs=6;
    for (auto &tile : f.device.tile_grid) {
      fpga::Element element;
      element.name="logic_slot_701";
      element.type=fpga::ELEMENT_LUT5;
      tile.tile_type->elements.push_back(element);
      element.name="extension_slot_703";
      element.type=fpga::ELEMENT_LUT1;
      tile.tile_type->elements.push_back(element);
      const bool occupied=std::any_of(f.tech.design.top.insts.begin(),
          f.tech.design.top.insts.end(),[&](const auto &inst) {return inst.tile.peer==&tile;});
      if (!occupied) f.cell("occupant_"+std::to_string(tile.coord.x)+"_"+
          std::to_string(tile.coord.y),tile.coord)->cnt_inputs=6;
      tile.invalidatePlacementCaches();
      require(tile.freeElementCount(fpga::ELEMENT_LUT5)==0 &&
                  tile.freeElementCount(fpga::ELEMENT_LUT1)==0,
              "wire-first fixture has an unintended free LUT position");
    }
    f.owner->outline.fixed=false;
    // Four labels cannot describe the eighteen-node reference. This case
    // requires a cheap probe to fail, real packing attempts to fail, and the
    // full wire fallback to recover the original position afterwards.
    if (mode==5) f.tech.route.moving_wire_probe_states=4;
  }
  std::ostringstream protected_before;
  fpga::dumpNetRouteHistory(*f.foreign,protected_before);
  f.audit();
  f.request=f.connect(*f.request_source,*f.request_sink);
  std::string failure, profile_text;
  {
    FanoutBudget budget(mode!=1);
    std::unique_ptr<RepairProfileCapture> profile;
    if(mode>=6) {
      // Example 1 repaired far more preserved/fixed endpoints than movable
      // endpoints. The separate fixed handler bypassed the preliminary source
      // probe and still flooded full GAP regions. All placement remains fixed
      // in these two cases; only wire recovery is permitted.
      profile=std::make_unique<RepairProfileCapture>();
      if(mode==7) f.tech.route.moving_wire_probe_states=4;
    }
    try { f.run({f.request_sink}); }
    catch(const std::runtime_error& error) { failure=error.what(); }
    if(profile) profile_text=profile->read();
  }
  require(f.complete(*f.request,*f.request_sink) && f.complete(*f.signal,*f.stable),
          "requester or independent sibling lost its complete route");
  require(f.complete(*f.foreign,*f.foreign_sink),"transit owner was not repaired completely");
  if(protected_takeoff) {
    require(!failure.empty() && fpga::hasRouteGaps(*f.route(*f.signal,*f.owner)),
            "protected Takeoff was cut or incomplete owner reported complete");
    std::ostringstream after; fpga::dumpNetRouteHistory(*f.foreign,after);
    require(after.str()==protected_before.str(),"protected source route changed");
  } else {
    require(failure.empty(),"focused fixed-endpoint Grounding failed: "+failure);
    require(f.complete(*f.signal,*f.owner),"fixed owner was not repaired");
    if (movable_owner) {
      if (mode==5)
        require(f.tech.route.failure_packing_trace &&
                    f.tech.route.failure_packing_net.find("owner_402")!=std::string::npos,
                "bounded wire probe bypassed the required packing/full-repair fallback");
      const auto *wire=f.route(*f.signal,*f.owner);
      require(f.owner->tile.peer==f.device.getTile(4,3) &&
                  f.owner->coord.x==4 && f.owner->coord.y==3 && f.owner->pos==0,
              "wire-first recovery relocated the movable endpoint");
      require(std::count_if(wire->begin(),wire->end(),[](const auto &w) {
                  return w.type==fpga::Wire::WIRE_CROSSBAR && w.jump>=0;
                })==8,"wire-first recovery did not use its eight-jump reference path");
      for (const auto &cut:std::vector<fpga::RouteCutNode>{
          {{4,1},fpga::CB_NODE_SRC,66},{{5,2},fpga::CB_NODE_DST,240},
          {{5,2},fpga::CB_NODE_SRC,284},{{4,3},fpga::CB_NODE_DST,66}}) {
        const auto selected=fpga::findNetOwnersByNode(
            *f.device.getTile(cut.tile.x,cut.tile.y),cut.type,cut.node);
        require(selected.size()==1 && selected.front().net==f.signal,
                "wire-first recovery did not own every required transit cut");
      }
    }
    const auto selected=fpga::findNetOwnersByNode(*f.device.getTile(4,1),fpga::CB_NODE_SRC,66);
    require(selected.size()==1 && selected.front().net==f.signal,
            "validated Grounding did not take the contested transit exit");
    const std::vector<int> bypass=movable_owner?std::vector<int>{281,282,283,285}:
                                               std::vector<int>{281,282,283};
    for(int src:bypass) {
      const auto c=src==281?fpga::Coord{4,1}:src==282?
          (movable_owner?fpga::Coord{5,1}:fpga::Coord{5,2}):
          src==283 && movable_owner?fpga::Coord{4,0}:fpga::Coord{5,4};
      require(f.device.getTile(c.x,c.y)->cb.src.jump.testBit(src),
              "displaced owner did not use its free repair bypass");
    }
  }
  if(mode!=1) require(f.tech.route.moving_stage && !f.tech.route.moving_sources_stage,
                     "zero-tick Fanouts budget did not reach Moving");
  for(auto* cell:{f.source,f.owner,f.stable,f.request_source,f.request_sink,f.foreign_source,f.foreign_sink})
    require(cell->outline.fixed==!(movable_owner && cell==f.owner) && cell->tile.peer &&
            cell->tile->coord.x==cell->coord.x && cell->tile->coord.y==cell->coord.y,
            "focused wire repair changed a fixed placement");
  require(f.tech.route.moving_focus_inst==nullptr,"wire-only preemption focus leaked");
  f.audit();
  if(mode>=6) {
    size_t exact_searches=0;
    std::istringstream lines(profile_text);
    for(std::string line;std::getline(lines,line);)
      if(line.find("REPAIR_PROFILE net="+f.signal->name+" ")!=std::string::npos &&
          line.find("phase=exact_gap_search ")!=std::string::npos) ++exact_searches;
    std::printf("fixed source probe full_fallback=%d exact_gap_phases=%zu\n",mode==7,exact_searches);
    require(mode==7 ? exact_searches>0 : exact_searches==0,
        mode==7 ? "undersized fixed source probe bypassed full GAP fallback" :
                  "fixed endpoint bypassed the cheap source probe and flooded full GAP search");
  }
  std::puts(protected_takeoff?"PROTECTED TAKEOFF CONTROL PASSED":
      movable_owner?"MOVABLE ENDPOINT WIRE-FIRST RECOVERY PASSED":"FIXED-ENDPOINT TRANSIT RECOVERY PASSED");
}
