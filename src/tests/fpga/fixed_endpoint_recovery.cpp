#include "Tech.h"
#include "Wire.h"

#include <cstdio>
#include <cstdlib>
#include <optional>
#include <initializer_list>
#include <limits>
#include <list>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void require(bool ok, const std::string &message) {
  if (!ok) throw std::runtime_error(message);
}

// Fixed endpoints do not make a retained routing gap irreparable.
// Phase 1 routes an owner and a surviving sibling. Phase 2's Basic Grounding
// takes a genuine transit JOINT and leaves the owner's prefix, gap and tail.
// Its only free repair bypass is through (1,2). A sub-nanosecond Fanouts budget
// converts to zero clock ticks, deterministically cancelling BEFORE repair;
// no sleeps, busy-bit fabrication, or test-side routing are involved.
// Moving has a fresh budget but used to reject the old fixed sink before it
// tried any routing. This repeats the incremental-run handoff mechanism, not
// the recorded chip's full geometry. All node and instance names are abstract.
struct Fixture {
  technology::Tech tech;
  fpga::Device &device = fpga::Device::current();
  Referable<rtl::Module> *parent, *primitive;
  std::list<Referable<rtl::Cell>> cells;
  std::list<Referable<pnr::RegBunch>> bunches;
  rtl::Inst *source, *owner, *stable, *request_source, *request_sink;
  rtl::Net *signal, *request=nullptr;

  explicit Fixture(bool with_bypass=true) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    device.size_width=5; device.size_height=3; device.grid_spec.size={5,3};
    device.tile_grid.resize(15); device.tile_types.reserve(15); device.cb_types.reserve(15);
    for (int y=0; y<3; ++y) for (int x=0; x<5; ++x) {
      auto &t=*device.getTile(x,y);
      auto &cb=device.cb_types.emplace_back();
      cb.type_id=cb.base_type_id=y*5+x;
      cb.name="matrix_"+std::to_string(701+y*5+x);
      auto &type=device.tile_types.emplace_back();
      type.name="resource_"+std::to_string(901+y*5+x);
      type.type=fpga::Tile::TILE_LUTS;
      type.pin_map.output_nodes[16].setBit(300);
      type.pin_map.nodes[{"LUT6","I0",0}].setBit(80);
      t.coord=t.cb_coord={x,y}; t.name=t.coord;
      t.full_name="tile_"+std::to_string(x)+"_"+std::to_string(y);
      t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
      name(t,fpga::CB_NODE_LOCAL,300); name(t,fpga::CB_NODE_LOCAL,80);
      cb.local_output_nodes.setBit(300); cb.local_input_nodes.setBit(80);
    }
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    tech.route.preserve_routes=true;
    parent=&tech.design.modules.emplace_back(); parent->name="fixed_recovery_top";
    parent->is_blackbox=false; parent->nets.reserve(64);
    primitive=&tech.design.modules.emplace_back(); primitive->name="logic_unit";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top"; tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent);
    tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("source_101",{0,1}); owner=cell("owner_102",{3,1});
    stable=cell("stable_103",{4,2});
    request_source=cell("request_source_201",{0,0});
    request_sink=cell("request_sink_203",{1,1});
    signal=connect(*source,*owner); connect(*source,*stable);

    edge({0,1},{1,1},300,100,200,true);
    edge({1,1},{2,1},200,246,20);
    auto &cross=*device.getTile(1,1)->cb_type;
    cross.dst_src[200].jump.clearBit(246);
    cross.dst_joint[200].joint.setBit(33);
    cross.joint_src[33].jump.setBit(246);
    name(*device.getTile(1,1),fpga::CB_NODE_JOINT,33);
    edge({2,1},{3,1},20,140,220); terminal({3,1},220);
    if(with_bypass) {
      edge({1,1},{1,2},200,180,260);
      edge({1,2},{2,1},260,181,20);
    }
    // This sibling must remain complete after the cut, just like the other
    // completed branches in the real failure. It does not use JOINT33.
    edge({0,1},{4,2},300,102,222,true); terminal({4,2},222);
    edge({0,0},{1,0},300,110,210,true); terminal({1,0},210);
    edge({1,0},{1,1},210,111,201);
    cross.dst_joint[201].joint.setBit(33);
    cross.joint_local[33].local.setBit(80);
    for (auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
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
    cb.dst_by_src[src].push_back(jump);
  }
  void terminal(fpga::Coord c,int dst) {
    device.getTile(c.x,c.y)->cb_type->dst_local[dst].local.setBit(80);
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
      std::vector<rtl::Net *> nets{signal};
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
    if (const char* value=std::getenv("SCALEPNR_ROUTE_FANOUT_TIMEOUT")) old=value;
    setenv("SCALEPNR_ROUTE_FANOUT_TIMEOUT",expire?"1e-99":"60",1);
  }
  ~FanoutBudget() {
    if (old) setenv("SCALEPNR_ROUTE_FANOUT_TIMEOUT",old->c_str(),1);
    else unsetenv("SCALEPNR_ROUTE_FANOUT_TIMEOUT");
  }
};
}

void fixed_endpoint_recovery_regression(int mode) {
  debug_level=1;
  const bool blocked=mode==2;
  Fixture f(!blocked);
  f.run({f.owner,f.stable});
  require(f.complete(*f.signal,*f.owner) && f.complete(*f.signal,*f.stable),
          "initial owner and stable branch did not route");
  require(f.device.getTile(1,1)->cb.joint.jump.testBit(33),
          "owner did not occupy the real contested transit");
  f.audit();
  f.request=f.connect(*f.request_source,*f.request_sink);
  std::string failure;
  {
    FanoutBudget budget(mode!=1);
    try { f.run({f.request_sink}); }
    catch(const std::runtime_error& error) { failure=error.what(); }
  }
  require(f.complete(*f.request,*f.request_sink),
          "Basic Grounding did not complete its replacement before the handoff");
  require(f.complete(*f.signal,*f.stable),
          "preemption damaged the independent stable branch");
  auto* owner=f.route(*f.signal,*f.owner);
  require(owner && !owner->empty(),"retained owner route disappeared");
  if (blocked) {
    require(!failure.empty() && fpga::hasRouteGaps(*owner),
            "blocked repair was silently accepted or lost its retained gap");
  } else {
    require(failure.empty(),"fixed owner was rejected without wire-only recovery: "+failure);
    require(f.complete(*f.signal,*f.owner),
            "free bypass did not repair the fixed owner's retained route");
    require(f.device.getTile(1,1)->cb.src.jump.testBit(180) &&
            f.device.getTile(1,2)->cb.src.jump.testBit(181),
            "owner recovery did not use the proven bypass");
  }
  if(mode!=1) require(f.tech.route.moving_stage && !f.tech.route.moving_sources_stage,
                     "zero-tick Fanouts budget did not exercise Moving handoff");
  require(f.source->outline.fixed && f.owner->outline.fixed && f.stable->outline.fixed &&
          f.request_source->outline.fixed && f.request_sink->outline.fixed,
          "recovery unfixed an endpoint");
  require(f.source->tile.peer==f.device.getTile(0,1) &&
          f.owner->tile.peer==f.device.getTile(3,1) &&
          f.stable->tile.peer==f.device.getTile(4,2) &&
          f.request_source->tile.peer==f.device.getTile(0,0) &&
          f.request_sink->tile.peer==f.device.getTile(1,1),
          "wire-only recovery relocated a fixed cell");
  f.audit();
  std::puts(blocked?"BLOCKED FIXED-ENDPOINT CONTROL PASSED":"FIXED-ENDPOINT RECOVERY PASSED");
}
