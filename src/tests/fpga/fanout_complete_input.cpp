#include "Tech.h"
#include "Wire.h"

#include <cstdio>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <list>
#include <sstream>
#include <stdexcept>

namespace {
using fpga::Coord;
using fpga::Wire;
using Route=std::vector<Wire>;
void require(bool ok,const std::string &s) { if(!ok)throw std::runtime_error(s); }
struct Node { Coord c; int kind,id; };

// Recorded run-5 failure: a retained gap ends at an input already reached by
// two complete siblings. Numeric search queued the sibling's first SRC but
// exhausted its frontier before expanding that SRC's landing. Preserve the
// complete 18-hop path, the failed 21-hop prefix, and the admitted frontier.
// All names are abstract. This is a reduced topology, not an entire device;
// omitted foreign congestion is represented by absent arcs, never ownerless bits.
struct Fixture {
  technology::Tech tech;
  fpga::Device &device=fpga::Device::current();
  std::list<Referable<rtl::Cell>> cells;
  Referable<rtl::Module> *parent,*primitive;
  rtl::Inst *source,*owner,*replica,*sink;
  rtl::Net *net;
  std::vector<Node> complete,partial;
  bool wrong_input;
  explicit Fixture(int mode):wrong_input(mode==2) {
    device.tile_grid.clear();device.tile_types.clear();device.cb_types.clear();
    device.route_wire_graph.clear();device.local_route_wire_mappings.clear();
    device.size_width=68;device.size_height=44;device.grid_spec.size={68,44};
    device.tile_grid.resize(68*44);device.tile_types.reserve(68*44);device.cb_types.reserve(68*44);
    for(int y=0;y<44;++y)for(int x=0;x<68;++x)device.getTile(x,y)->coord={x,y};
    tech.init();tech.design.GND=nullptr;tech.design.VCC=nullptr;
    parent=&tech.design.modules.emplace_back();parent->name="input_reuse_top";
    parent->is_blackbox=false;parent->nets.reserve(4);
    primitive=&tech.design.modules.emplace_back();primitive->name="abstract_unit";
    primitive->is_blackbox=true;primitive->parent_ref.set(parent);
    tech.design.top_cell.name="top";tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent);tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("driver_278",{50,8},0,true);
    owner=cell("sink_5827",{12,18},0,false);
    replica=cell("sink_5819",{12,18},1,false);
    sink=cell("sink_5801",{12,18},2,false);
    net=&parent->nets.emplace_back();net->name="signal_278";net->designators.push_back(1);
    source->conns[1].port_ref->designator=1;
    for(auto *s:{owner,replica,sink}) {s->conns[0].port_ref->designator=1;s->conns[0].set(&source->conns[1]);}
    std::ifstream paths(std::filesystem::path(__FILE__).replace_filename("fanout_complete_input_paths.txt"));
    require(bool(paths),"missing recorded paths");
    std::string role;Node n;
    while(paths>>role>>n.c.x>>n.c.y>>n.kind>>n.id)(role=="complete"?complete:partial).push_back(n);
    require(paths.eof()&&complete.size()==41&&partial.size()==44,"incomplete recorded paths");
    if(wrong_input)complete.back().id=41;
    if(mode!=1) {
      std::ifstream input(std::filesystem::path(__FILE__).replace_filename("fanout_complete_input_frontier.txt"));
      require(bool(input),"missing recorded frontier");Node a,b;size_t count=0;
      while(input>>a.c.x>>a.c.y>>a.kind>>a.id>>b.c.x>>b.c.y>>b.kind>>b.id) {
        if(!(wrong_input&&b.c.x==12&&b.c.y==18&&b.kind==fpga::CB_NODE_LOCAL&&b.id==40))arc(a,b);
        ++count;
      }
      require(input.eof()&&count==67948,"incomplete recorded frontier");
    }
    for(size_t i=1;i<complete.size();++i)arc(complete[i-1],complete[i]);
    for(size_t i=1;i+1<partial.size();++i)arc(partial[i-1],partial[i]);
    // These raw terminal arcs exist in the failure snapshot, but cannot appear
    // in its allowed frontier: JOINT63 already has the completed sibling's
    // JOINT43 input. Keep them for the endpoint topology/mapping checks.
    for(int dst:{20,24,65,193,244,246,257,261,545,737,1025,3073,3617,3810,3841,3846})
      arc({{12,18},fpga::CB_NODE_DST,dst},{{12,18},fpga::CB_NODE_JOINT,63});
    for(auto &cb:device.cb_types)cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
    publish(*owner,makeRoute(complete,false,0));
    auto shared=makeRoute(complete,false,1);
    for(auto &w:shared){w.shared=true;w.owns_dst=w.owns_landing=false;}
    publish(*replica,std::move(shared));
    publish(*sink,makeRoute(partial,true,2));
  }
  fpga::Tile &tile(Coord c) {
    auto &t=*device.getTile(c.x,c.y);if(t.cb_type)return t;
    auto &cb=device.cb_types.emplace_back();cb.type_id=device.cb_types.size()-1;cb.base_type_id=0;
    cb.name="matrix_"+std::to_string(cb.type_id);
    cb.local_output_nodes.setBit(154);
    cb.local_input_nodes.setBit(40);cb.local_input_nodes.setBit(41);
    auto &type=device.tile_types.emplace_back();type.name="resource_"+std::to_string(cb.type_id);
    type.type=fpga::Tile::TILE_LUTS;type.pin_map.output_nodes[16].setBit(154);
    for(int p=0;p<3;++p)type.pin_map.nodes[{"FDRE","CE",p}].setBit(wrong_input&&p<2?41:40);
    t.cb_coord=c;t.name=c;t.full_name="tile_"+std::to_string(c.x)+"_"+std::to_string(c.y);
    t.type=fpga::Tile::TILE_LUTS;t.tile_type=&type;t.cb_type=t.cb.type=&cb;
    return t;
  }
  void name(const Node &n) {
    tile(n.c).cb_type->rememberNodeName(static_cast<fpga::CBNodeNameType>(n.kind),n.id,
        "node_"+std::to_string(unsigned(n.id)*2654435761U+unsigned(n.kind)));
  }
  void arc(const Node &a,const Node &b) {
    name(a);name(b);auto &cb=*tile(a.c).cb_type;
    if(a.kind==fpga::CB_NODE_SRC&&b.kind==fpga::CB_NODE_DST) {
      for(const auto &j:cb.dst_by_src[a.id])if(j.delta.x==b.c.x-a.c.x&&j.delta.y==b.c.y-a.c.y&&j.dsts.jump.testBit(b.id))return;
      fpga::CBType::ResolvedJump j{};j.delta=b.c-a.c;j.target_tile_coord=true;j.target_cb_type_id=0;
      j.dsts.jump.setBit(b.id);cb.dst_by_src[a.id].push_back(j);return;
    }
    if(a.kind==fpga::CB_NODE_LOCAL&&(a.c.x!=b.c.x||a.c.y!=b.c.y)) {
      fpga::CBType::ResolvedLocal j{};j.delta=b.c-a.c;j.target_cb_type_id=0;
      j.target_node_type=static_cast<fpga::CBNodeNameType>(b.kind);j.target_nodes.setBit(b.id);
      cb.local_by_local[a.id].push_back(j);return;
    }
    require(a.c.x==b.c.x&&a.c.y==b.c.y,"unexpected cross-Tile arc");
    if(a.kind==0&&b.kind==3)cb.local_src[a.id].jump.setBit(b.id);
    else if(a.kind==0&&b.kind==0)cb.local_local[a.id].local.setBit(b.id);
    else if(a.kind==0&&b.kind==1)cb.local_joint[a.id].joint.setBit(b.id);
    else if(a.kind==4&&b.kind==3)cb.dst_src[a.id].jump.setBit(b.id);
    else if(a.kind==4&&b.kind==0)cb.dst_local[a.id].local.setBit(b.id);
    else if(a.kind==4&&b.kind==1)cb.dst_joint[a.id].joint.setBit(b.id);
    else if(a.kind==3&&b.kind==1)cb.src_joint[a.id].joint.setBit(b.id);
    else if(a.kind==1&&b.kind==3)cb.joint_src[a.id].jump.setBit(b.id);
    else if(a.kind==1&&b.kind==0)cb.joint_local[a.id].local.setBit(b.id);
    else if(a.kind==1&&b.kind==1)cb.joint_joint[a.id].joint.setBit(b.id);
    else require(false,"unexpected arc kind");
    cb.rememberConnName(static_cast<fpga::CBNodeNameType>(a.kind),a.id,
        static_cast<fpga::CBNodeNameType>(b.kind),b.id,
        *cb.nodeName(static_cast<fpga::CBNodeNameType>(a.kind),a.id),
        *cb.nodeName(static_cast<fpga::CBNodeNameType>(b.kind),b.id));
  }
  rtl::Inst *cell(const std::string &label,Coord c,int pos,bool output) {
    auto &type=cells.emplace_back();type.name=label;type.type=output?"LUT6":"FDRE";
    type.module_ref.set(primitive);type.ports.reserve(2);
    for(auto d:{rtl::Port::PORT_IN,rtl::Port::PORT_OUT}) {
      auto &p=type.ports.emplace_back();p.type=d;p.designator=-1;
      p.name=d==rtl::Port::PORT_IN?(output?"I0":"CE"):(output?"O":"Q");
    }
    auto &i=tech.design.top.insts.emplace_back();i.cell_ref.set(&type);i.parent_ref.set(&tech.design.top);
    i.depth=1;i.coord=c;i.pos=pos;i.outline.fixed=true;i.cnt_inputs=i.cnt_outputs=1;i.conns.reserve(2);
    for(auto &p:type.ports){auto &conn=i.conns.emplace_back();conn.port_ref.set(&p);conn.inst_ref.set(&i);}
    tile(c).assign(&i);return &i;
  }
  Wire pin(const Node &n,bool output,int pos) {
    name(n);Wire w;w.type=Wire::WIRE_TILE_PIN;w.from=w.to=w.resource=n.c;w.local=n.id;w.pos=pos;
    w.pin_dir=output?fpga::TILE_PIN_OUTPUT:fpga::TILE_PIN_INPUT;
    w.cell_type=output?"LUT6":"FDRE";w.port=output?"O":"CE";w.shared=output;return w;
  }
  Route makeRoute(const std::vector<Node> &path,bool gap,int pos) {
    Route r{pin(path.front(),true,0)};
    for(size_t i=0;i+1<path.size();) {
      Wire w;w.from=path[i].c;w.to=path[i+1].c;
      if(gap&&i+2==path.size()) {
        w.type=Wire::WIRE_GAP;w.from_node_type=path[i].kind;w.from_node=path[i].id;
        w.to_node_type=path[i+1].kind;w.to_node=path[i+1].id;++i;
      } else if(path[i+1].kind==fpga::CB_NODE_SRC&&i+2<path.size()&&path[i+2].kind==fpga::CB_NODE_DST) {
        w.pos=i==0?0:1;w.local=path[i].id;w.jump=w.route_jump=path[i+1].id;
        w.to=path[i+2].c;w.dst=path[i+2].id;w.owns_landing=true;i+=2;
      } else {
        w.type=Wire::WIRE_ROUTE_EDGE;w.from_node_type=path[i].kind;w.from_node=path[i].id;
        w.to_node_type=path[i+1].kind;w.to_node=path[i+1].id;++i;
      }
      r.push_back(w);
    }
    r.push_back(pin(path.back(),false,pos));return r;
  }
  void publish(rtl::Inst &target,Route r) {
    // Reconstruct leases from recorded fragments, each with a real binding.
    auto lease=[&](Coord c,int kind,int id) {
      // The recorded source LOCAL is an unleased source reference.
      if(c.x==50&&c.y==8&&kind==0&&id==154)return;
      auto &t=tile(c);if(kind==0)t.cb.local.local.setBit(id);
      else if(kind==1)t.cb.joint.jump.setBit(id);
      else if(kind==3)t.cb.src.jump.setBit(id);else if(kind==4)t.cb.dst.jump.setBit(id);
    };
    for(const auto &w:r) {
      if(w.type==Wire::WIRE_TILE_PIN&&w.pin_dir==fpga::TILE_PIN_INPUT){lease(w.from,0,w.local);tile(w.from).pin_state.leased_nodes.setBit(w.local);}
      else if(w.type==Wire::WIRE_ROUTE_EDGE){lease(w.from,w.from_node_type,w.from_node);lease(w.to,w.to_node_type,w.to_node);}
      else if(w.type==Wire::WIRE_CROSSBAR){lease(w.from,w.pos?4:0,w.local);lease(w.from,3,w.jump);lease(w.to,4,w.dst);}
    }
    target.wires.push_back(std::move(r));
    auto index=fpga::attachNetRoute(*net,target,0,source,&target,"O","CE",target.makeName()+".signal");
    fpga::registerNetRouteTiles(*net,target.wires.front(),index);
  }
  void audit() {
    for(auto &t:device.tile_grid)if(t.cb_type) {
      std::ostringstream out;auto a=fpga::auditTileCongestion(t,out,{net});
      require(!a.orphan_leases&&!a.missing_leases&&!a.missing_registrations&&!a.stale_registrations&&!a.conflicting_driver_nodes,out.str());
    }
  }
};
}

void fanout_complete_input_regression(int mode) {
  debug_level=1;Fixture f(mode);f.audit();
  require(f.sink->tile->getPinNodes("FDRE","CE",2).testBit(40),"fixture lost target pin mapping");
  if(mode!=2)require(f.sink->tile->cb_type->dsts_reaching_local[40].jump.firstSetBit()>=0,
      "fixture lost terminal entry topology");
  require(fpga::isRouteComplete(f.owner->wires.front())&&fpga::isRouteComplete(f.replica->wires.front()),"missing complete siblings");
  const Route owner_before=f.owner->wires.front(),replica_before=f.replica->wires.front();
  std::ostringstream before;fpga::dumpNetRouteHistory(*f.net,before);
  require(!fpga::repairNetRouteGaps(*f.net,2,[]{return true;},32768,200),"cancelled repair succeeded");
  std::ostringstream cancelled;fpga::dumpNetRouteHistory(*f.net,cancelled);
  require(before.str()==cancelled.str(),"cancelled repair mutated routes");
  const auto &b=f.net->routes[2];
  pnr::RouteDesign::RouteTask task{.from=f.source,.to=f.sink,.net=f.net,
      .from_port="O",.to_port="CE",.net_name=b.route_name,.fanout=true,.endpoints_prepared=true};
  const auto start=std::chrono::steady_clock::now();
  const bool repaired=f.tech.route.repairTaskGaps(task,2);
  const double repair_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  f.audit();
  std::printf("complete-input fixture: mode=%d repaired=%d repair_ms=%.3f\n",mode,repaired,repair_ms);
  auto same_path=[](const Route &left,const Route &right) {
    if(left.size()!=right.size())return false;
    for(size_t i=0;i<left.size();++i) {
      const auto &a=left[i],&b=right[i];
      if(a.type!=b.type||a.from.x!=b.from.x||a.from.y!=b.from.y||a.to.x!=b.to.x||a.to.y!=b.to.y||
          a.local!=b.local||a.jump!=b.jump||a.dst!=b.dst||a.joint!=b.joint||a.joint2!=b.joint2||
          a.from_node_type!=b.from_node_type||a.from_node!=b.from_node||
          a.to_node_type!=b.to_node_type||a.to_node!=b.to_node)return false;
    }
    return true;
  };
  require(same_path(owner_before,f.owner->wires.front())&&same_path(replica_before,f.replica->wires.front()),
      "repair changed a completed sibling's physical path");
  if(mode==2) {
    require(!repaired,"different physical input was reused");
    std::ostringstream after;fpga::dumpNetRouteHistory(*f.net,after);
    require(before.str()==after.str(),"failed repair mutated retained routes");return;
  }
  require(repaired,"BUG: complete sibling input lost behind bounded gap search");
  auto &result=f.net->routes[2];auto &route=result.owner->wires[result.route_index];
  require(fpga::isRouteComplete(route),"repair left a gap");
  require(f.net->routes.size()==3&&same_path(route,f.owner->wires.front()),"repair did not reuse the complete input path");
  require(fpga::isRouteComplete(f.owner->wires.front())&&fpga::isRouteComplete(f.replica->wires.front()),"repair damaged siblings");
  require(!f.device.getTile(11,15)->cb.dst.jump.testBit(3619),"abandoned private prefix lease remains");
  require(f.source->coord==Coord{50,8}&&f.sink->coord==Coord{12,18}&&f.sink->pos==2,"fixed endpoint moved");
  std::ostringstream completed;fpga::dumpNetRouteHistory(*f.net,completed);
  const auto stored=f.sink->wires.size();
  require(f.tech.route.repairTaskGaps(task,2),"complete route failed an idempotent repair");
  std::ostringstream repeated;fpga::dumpNetRouteHistory(*f.net,repeated);
  require(stored==f.sink->wires.size()&&completed.str()==repeated.str(),"complete route repair was not a no-op");
  for(size_t i=0;i<f.net->routes.size();++i)require(fpga::unrouteNetRoute(*f.net,i),"teardown failed");
  f.audit();
}
