#pragma once

#include "Tech.h"
#include "route/RouteDiagnostics.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <list>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace routing_tests {

inline void require(bool value, const std::string &message) {
  if (!value) throw std::runtime_error(message);
}

// A real source-recovery trace retried the same register CE placement for
// different output exits. Thousands of complete reverse input walks dominated
// the stage. Keep its essential geometry: an existing control tree, many
// output alternatives at an unusable placement, and a legal nearby placement.
// Every wire and resource name is abstract; all leases have real route owners.
struct SourceInputFixture {
  technology::Tech tech;
  fpga::Device &device=fpga::Device::current();
  Referable<rtl::Module> *parent=nullptr, *primitive=nullptr;
  std::list<Referable<rtl::Cell>> cells;
  rtl::Inst *source=nullptr, *sink=nullptr, *control=nullptr;
  rtl::Net *signal=nullptr, *enable=nullptr;

  explicit SourceInputFixture(bool output_conflict,int current_wire=0,size_t net_capacity=4, int width=8, int requested_height=0);

  void capturedWideWire();
  void capturedResolvedLocalWire();
  void capturedDispatcherWire();
  std::pair<rtl::Inst*,rtl::Inst*> installCEFrontier(
      bool connect_branch, fpga::Coord candidate_coord={1,0});

  void resolvedLocal(fpga::Coord from,int local,fpga::Coord to,fpga::CBNodeNameType kind,int node) {
    name(tile(from),fpga::CB_NODE_LOCAL,local); name(tile(to),kind,node);
    fpga::CBType::ResolvedLocal link;
    link.delta=to-from; link.target_cb_type_id=tile(to).cb_type->type_id;
    link.target_node_type=kind; link.target_nodes.setBit(node);
    tile(from).cb_type->local_by_local[local].push_back(link);
  }

  void capturedWire();

  fpga::Tile &tile(fpga::Coord c) { return *device.getTile(c.x,c.y); }
  static void name(fpga::Tile &t,fpga::CBNodeNameType kind,int n) {
    const std::string label="node_"+std::to_string(int(kind))+"_"+std::to_string(n);
    t.cb_type->rememberNodeName(kind,n,label);
  }
  fpga::Wire edge(fpga::Coord a,fpga::Coord b,int input,int src,int dst,bool takeoff=false) {
    auto &from=tile(a); auto &to=tile(b); auto &cb=*from.cb_type;
    name(from,takeoff?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST,input);
    name(from,fpga::CB_NODE_SRC,src); name(to,fpga::CB_NODE_DST,dst);
    if(takeoff) cb.local_src[input].jump.setBit(src); else cb.dst_src[input].jump.setBit(src);
    fpga::CBType::ResolvedJump jump{}; jump.delta=b-a; jump.target_tile_coord=true;
    jump.target_cb_type_id=to.cb_type->type_id; jump.dsts.jump.setBit(dst);
    auto &targets=cb.dst_by_src[src];
    if(targets.empty()) targets.push_back(jump);
    else require(targets.size()==1 && targets.front().delta.x==jump.delta.x &&
        targets.front().delta.y==jump.delta.y && targets.front().dsts.jump==jump.dsts.jump,
        "one source node has incompatible physical landings");
    fpga::Wire w; w.from=a; w.to=b; w.local=input; w.jump=src; w.dst=dst;
    w.pos=takeoff?0:1; w.owns_landing=true; return w;
  }
  fpga::Wire terminal(fpga::Coord c,int dst,int local) {
    auto &t=tile(c); name(t,fpga::CB_NODE_DST,dst); name(t,fpga::CB_NODE_LOCAL,local);
    t.cb_type->dst_local[dst].local.setBit(local);
    fpga::Wire w; w.from=w.to=c; w.local=dst; w.jump=w.dst=-1; w.pos=1; return w;
  }
  static fpga::Wire pin(fpga::Coord c,int local,bool output) {
    fpga::Wire w; w.type=fpga::Wire::WIRE_TILE_PIN; w.from=w.to=c; w.local=local;
    w.pin_dir=output?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT; return w;
  }
  rtl::Inst *cell(const std::string &label,const std::string &kind,
                 std::initializer_list<std::pair<const char*,bool>> ports,fpga::Coord c,
                 int requested_pos=-1) {
    auto &type=cells.emplace_back(); type.name=label; type.type=kind; type.module_ref.set(primitive);
    type.ports.reserve(ports.size());
    for(auto [name,output]:ports) {
      auto &p=type.ports.emplace_back(); p.name=name;
      p.type=output?rtl::Port::PORT_OUT:rtl::Port::PORT_IN; p.designator=-1;
    }
    auto &i=tech.design.top.insts.emplace_back(); i.cell_ref.set(&type);
    i.parent_ref.set(&tech.design.top); i.depth=1; i.coord=c; i.conns.reserve(ports.size());
    for(auto &p:type.ports) {
      auto &conn=i.conns.emplace_back(); conn.port_ref.set(&p); conn.inst_ref.set(&i);
      if(p.type==rtl::Port::PORT_IN) ++i.cnt_inputs; else ++i.cnt_outputs;
    }
    const int pos=requested_pos>=0?requested_pos:kind=="LUT6"?3:kind=="MUXF7"?1:0;
    require(tile(c).tryAddAt(&i,pos)==pos,"cannot place fixture endpoint '"+label+
        "' type="+kind+" pos="+std::to_string(pos)); return &i;
  }
  static rtl::Conn &port(rtl::Inst &i,const std::string &name) {
    for(auto &c:i.conns) if(c.port_ref->name==name) return c;
    throw std::runtime_error("missing fixture port");
  }
  rtl::Net *connect(rtl::Inst &a,const std::string &out,rtl::Inst &b,const std::string &in) {
    auto &net=parent->nets.emplace_back(); const int id=parent->nets.size();
    net.name="signal_"+std::to_string(id); net.designators.push_back(id);
    port(a,out).port_ref->designator=port(b,in).port_ref->designator=id;
    port(b,in).set(&rtl::Conn::fromBase(port(a,out))); return &net;
  }
  void audit() {
    for(auto &t:device.tile_grid) {
      std::ostringstream out;
      const auto a=fpga::auditTileCongestion(t,out,{signal,enable});
      require(!a.orphan_leases && !a.missing_leases && !a.missing_registrations &&
          !a.stale_registrations && !a.conflicting_driver_nodes,out.str());
    }
  }
  void auditAll() {
    std::vector<rtl::Net*> nets;
    for(auto &net:parent->nets) nets.push_back(&net);
    for(auto &t:device.tile_grid) {
      std::ostringstream out;
      const auto a=fpga::auditTileCongestion(t,out,nets);
      require(!a.orphan_leases && !a.missing_leases && !a.missing_registrations &&
          !a.stale_registrations && !a.conflicting_driver_nodes,out.str());
    }
  }
};

} // namespace routing_tests
