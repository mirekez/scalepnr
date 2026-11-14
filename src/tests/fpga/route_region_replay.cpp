#include "SourceInputFixture.h"
#include "RepairStatistics.h"
#include <json/json.h>
#include <filesystem>
#include <fstream>
#include <set>
#include <tuple>

namespace {
using namespace routing_tests;
using C = fpga::Coord;
using W = fpga::Wire;
using Node = std::tuple<int,int,int,int>;

// Captured Example 1 run 076, chunk 6, at the Moving deadline. The region
// contains the entire 49-node diagnostic witness AND all alternative numeric
// edges inside [9..69] x [0..17]. Crossing edges are explicitly omitted.
// All 8,959 live bindings are retained, including out-of-region fragments:
// a transit cut must still discover every dependent branch and its real owner.
// This tests generic wire repair with captured placement/pin reservations, not
// Moving: Moving now resumes angle-led tracing and recursively moves inputs.
// Generic gap repair still exercises the full captured ownership hierarchy. It does not
// recreate packing models or the preceding ten minutes of whole-chip work.
// Every instance, net, matrix and node receives an abstract pattern name.
struct Region {
  technology::Tech tech;
  fpga::Device& device = fpga::Device::current();
  std::list<Referable<rtl::Cell>> cells;
  std::vector<rtl::Inst*> insts;
  std::vector<rtl::Net*> nets;
  std::vector<C> selected;
  std::vector<Node> witness;
  pnr::RouteDesign::RouteTask task;
  size_t trigger_index=0, fragments=0;

  static int n(const Json::Value& v,int i) { return v[i].asInt(); }
  static C co(const Json::Value& v,int i=0) { return {n(v,i),n(v,i+1)}; }
  static NodeMask mask(const Json::Value& v) {
    NodeMask m;for(const auto& b:v) {require(b.asInt()>=0 && b.asInt()<CB_MAX_NODES,"invalid captured node");m.setBit(b.asInt());}return m;
  }
  fpga::Tile& tile(C c) {auto* t=device.getTile(c.x,c.y);require(t!=nullptr,"captured coordinate outside grid");return *t;}
  static void name(fpga::CBType& cb,int kind,int id) {
    cb.rememberNodeName(fpga::CBNodeNameType(kind),id,"pattern_"+std::to_string(kind)+"_"+std::to_string(id));
  }
  static void table(fpga::CBType& cb,int kind,int from,int to) {
    static constexpr int kinds[][2]={{0,3},{0,1},{0,0},{4,3},{4,1},{4,0},{1,3},{1,1},{1,0},{3,1}};
    name(cb,kinds[kind][0],from);name(cb,kinds[kind][1],to);
    switch(kind) {
      case 0:cb.local_src[from].jump.setBit(to);break;
      case 1:cb.local_joint[from].joint.setBit(to);break;
      case 2:cb.local_local[from].local.setBit(to);break;
      case 3:cb.dst_src[from].jump.setBit(to);break;
      case 4:cb.dst_joint[from].joint.setBit(to);break;
      case 5:cb.dst_local[from].local.setBit(to);break;
      case 6:cb.joint_src[from].jump.setBit(to);break;
      case 7:cb.joint_joint[from].joint.setBit(to);break;
      case 8:cb.joint_local[from].local.setBit(to);break;
      case 9:cb.src_joint[from].joint.setBit(to);break;
    }
  }
  explicit Region() {
    std::ifstream file(std::filesystem::path(__FILE__).parent_path()/"moving_region_snapshot.json");
    Json::Value data;Json::Reader reader;
    require(file.good() && reader.parse(file,data),"cannot read captured Tile region");
    require(data["routes"].size()==8959 && data["wires"].size()==113751 && data["witness"].size()==49,"snapshot population changed");
    device.tile_grid.clear();device.cb_types.clear();device.tile_types.clear();
    device.route_wire_graph.clear();device.local_route_wire_mappings.clear();
    device.size_width=n(data["size"],0);device.size_height=n(data["size"],1);
    device.grid_spec.size={device.size_width,device.size_height};
    device.tile_grid.resize(device.size_width*device.size_height);
    device.cb_types.reserve(data["types"].size()+data["tiles"].size()+1);
    device.tile_types.resize(device.tile_grid.size());
    for(auto& t:device.tile_grid) {
      const auto index=&t-device.tile_grid.data();t.coord=t.cb_coord={int(index%device.size_width),int(index/device.size_width)};
      t.tile_type=&device.tile_types[index];t.tile_type->name="pattern_resources_"+std::to_string(index);
      t.type=fpga::Tile::TILE_LUTS;t.tile_type->type=t.type;t.full_name="pattern_tile_"+std::to_string(index);
    }
    for(const auto& spec:data["types"]) {
      auto& cb=device.cb_types.emplace_back();cb.type_id=cb.base_type_id=device.cb_types.size()-1;
      cb.name="pattern_matrix_"+std::to_string(cb.type_id);
      for(int k=0;k<10;++k)for(const auto& row:spec[k])for(const auto& bit:row[1])table(cb,k,n(row,0),bit.asInt());
      cb.local_input_nodes=mask(spec[10]);cb.local_output_nodes=mask(spec[11]);cb.valid_dst_nodes=mask(spec[12]);
    }
    auto& empty=device.cb_types.emplace_back();empty.type_id=empty.base_type_id=device.cb_types.size()-1;empty.name="pattern_boundary";
    const auto empty_id=empty.type_id;
    for(const auto& row:data["tiles"]) {
      auto& t=tile(co(row));t.cb_coord=co(row,2);
      if(n(row,4)>=0) {
        device.cb_types.push_back(device.cb_types[n(row,4)]);auto& cb=device.cb_types.back();
        cb.type_id=cb.base_type_id=device.cb_types.size()-1;cb.name="pattern_matrix_"+std::to_string(cb.type_id);
        t.cb_type=&cb;selected.push_back(t.coord);
      } else t.cb_type=&device.cb_types[empty_id];
      t.cb.type=t.cb_type;
      t.cb.local.local=mask(row[5][0]);t.cb.joint.jump=mask(row[5][1]);
      t.cb.src.jump=mask(row[5][2]);t.cb.dst.jump=mask(row[5][3]);
      t.pin_state.leased_nodes=mask(row[6]);t.cb.src_deadend.jump=mask(row[7]);t.incoming_dst_nodes=mask(row[8]);
    }
    require(selected.size()==901,"captured region size changed");
    for(const auto& e:data["links"]) {
      auto& a=tile(co(e));auto& b=tile(co(e,4));const int src=n(e,3),dst=n(e,7);
      name(*a.cb_type,n(e,2),src);name(*b.cb_type,n(e,6),dst);
      if(n(e,2)==3) {
        fpga::CBType::ResolvedJump link;link.delta=b.coord-a.coord;link.target_tile_coord=true;
        link.target_cb_type_id=b.cb_type->type_id;link.dsts.jump.setBit(dst);a.cb_type->dst_by_src[src].push_back(link);
      } else {
        fpga::CBType::ResolvedLocal link;link.delta=b.coord-a.coord;link.target_cb_type_id=b.cb_type->type_id;
        link.target_node_type=fpga::CBNodeNameType(n(e,6));link.target_nodes.setBit(dst);a.cb_type->local_by_local[src].push_back(link);
      }
    }
    for(auto& cb:device.cb_types)cb.rebuildOutgoingSrcs();
    // Preserve the captured incoming mask, even at the closed region boundary.
    tech.init();tech.design.GND=tech.design.VCC=nullptr;
    auto& parent=tech.design.modules.emplace_back();parent.name="pattern_top";parent.nets.reserve(data["nets"].size());
    auto& primitive=tech.design.modules.emplace_back();primitive.name="pattern_primitive";primitive.is_blackbox=true;primitive.parent_ref.set(&parent);
    tech.design.top_cell.name=tech.design.top_cell.type="pattern_top";tech.design.top_cell.module_ref.set(&parent);tech.design.top.cell_ref.set(&tech.design.top_cell);
    for(const auto& row:data["insts"]) {
      auto& cell=cells.emplace_back();cell.name="pattern_cell_"+std::to_string(insts.size());cell.type=row[0].asString();cell.module_ref.set(&primitive);
      if(n(row,9))cell.attributes["scalepnr_passthrough"]="source";
      cell.ports.reserve(row[10].size());
      for(const auto& p:row[10]) {auto& port=cell.ports.emplace_back();port.name=p[0].asString();port.type=decltype(port.type)(n(p,1));port.designator=n(p,2);}
      auto& inst=tech.design.top.insts.emplace_back();inst.cell_ref.set(&cell);inst.parent_ref.set(&tech.design.top);inst.depth=1;
      inst.coord=co(row,1);inst.pos=n(row,3);inst.outline.fixed=n(row,6);inst.pnr_completed=n(row,7);inst.pnr_pending=n(row,8);
      if(inst.coord.x>=0 && inst.coord.y>=0) {auto& t=tile(inst.coord);inst.tile.set(&device.tile_grid[inst.coord.y*device.size_width+inst.coord.x]);t.cb_coord=co(row,4);}
      inst.conns.reserve(cell.ports.size());
      for(auto& p:cell.ports) {auto& conn=inst.conns.emplace_back();conn.port_ref.set(&p);conn.inst_ref.set(&inst);if(p.type==rtl::Port::PORT_IN)++inst.cnt_inputs;else ++inst.cnt_outputs;}
      insts.push_back(&inst);
    }
    for(Json::ArrayIndex i=0;i<data["insts"].size();++i)for(const auto& p:data["insts"][i][10])if(n(p,3)>=0) {
      auto& input=SourceInputFixture::port(*insts[i],p[0].asString());auto& output=SourceInputFixture::port(*insts[n(p,3)],p[4].asString());
      input.set(&rtl::Conn::fromBase(output));
    }
    for(const auto& p:data["nets"]) {auto& net=parent.nets.emplace_back();net.name="pattern_net_"+std::to_string(nets.size());net.route_protected=p.asBool();nets.push_back(&net);}
    std::vector<W> wires;wires.reserve(data["wires"].size());
    for(const auto& v:data["wires"]) {
      W w;w.from=co(v);w.to=co(v,2);w.resource=co(v,4);w.type=W::Type(n(v,6));w.local=n(v,7);w.pos=n(v,8);w.jump=n(v,9);w.route_jump=n(v,10);w.dst=n(v,11);w.joint=n(v,12);w.joint2=n(v,13);
      w.from_node_type=n(v,14);w.from_node=n(v,15);w.to_node_type=n(v,16);w.to_node=n(v,17);w.resource_node=n(v,18);w.pin_dir=n(v,19);w.shared=n(v,20);w.owns_dst=n(v,21);w.owns_landing=n(v,22);wires.push_back(w);
    }
    size_t ri=0;
    for(const auto& r:data["routes"]) {
      auto* net=nets[n(r,0)];auto* from=insts[n(r,1)];auto* to=insts[n(r,2)];auto* owner=insts[n(r,3)];size_t slot=n(r,4);
      if(owner->wires.size()<=slot)owner->wires.resize(slot+1);
      auto& path=owner->wires[slot];require(path.empty(),"duplicate route storage");
      for(const auto& id:r[8])path.push_back(wires[id.asUInt()]);
      fragments+=path.size();
      std::string route_name="pattern_route_"+std::to_string(ri++);
      auto index=fpga::attachNetRoute(*net,*owner,slot,from,to,r[5].asString(),r[6].asString(),route_name);
      require(net->routeId(index)==r[7].asUInt64(),"captured binding identity changed");
      fpga::registerNetRouteTiles(*net,path,index);
      tech.route.indexSourceRoute(net,from,r[5].asString());
      if(n(r,9)) {task={from,to,net,r[5].asString(),r[6].asString(),route_name};trigger_index=index;}
    }
    for(const auto& row:data["tiles"]) {
      auto& t=tile(co(row));
      for(const auto& p:row[9])t.input_local_reservations[n(p,0)]=&SourceInputFixture::port(*insts[n(p,1)],p[2].asString());
      for(const auto& p:row[10])t.input_joint_reservations.emplace_back(&SourceInputFixture::port(*insts[n(p,0)],p[1].asString()),mask(p[2]));
      t.input_joint_reservations_initialized=true;
    }
    for(const auto& v:data["witness"])witness.emplace_back(n(v,0),n(v,1),n(v,2),n(v,3));
    task.fanout=task.endpoints_prepared=true;
    auto& router=tech.route;router.complete_routes_only=router.preserve_routes=true;
    router.moving_stage=false;
    router.moving_sources_stage=router.route_deadends_enabled=false;router.route_preemption_enabled=true;
    router.route_recursion_budget=router.iteration_limit=router.route_suffix_depth_limit=200;
  }
  bool edge(Node a,Node b) {
    auto[x,y,k,n]=a;auto[u,v,l,m]=b;auto& t=tile({x,y});const auto& cb=*t.cb_type;
    if(k==3 && l==4) {
      for(const auto& target:device.resolveJumpTargets(t,n))if(target.tile->coord==C{u,v} && target.dst_node==m)return true;
      return false;
    }
    if(k==0)for(const auto& target:device.resolveLocalTargets(t,n))
      if(target.tile->coord==C{u,v} && int(target.node_type)==l && target.node==m)return true;
    if(x!=u || y!=v)return false;
    if(k==0 && l==3)return cb.local_src[n].jump.testBit(m);
    if(k==0 && l==1)return cb.local_joint[n].joint.testBit(m);
    if(k==0 && l==0)return cb.local_local[n].local.testBit(m);
    if(k==4 && l==3)return cb.dst_src[n].jump.testBit(m);
    if(k==4 && l==1)return cb.dst_joint[n].joint.testBit(m);
    if(k==4 && l==0)return cb.dst_local[n].local.testBit(m);
    if(k==1 && l==3)return cb.joint_src[n].jump.testBit(m);
    if(k==1 && l==1)return cb.joint_joint[n].joint.testBit(m);
    if(k==1 && l==0)return cb.joint_local[n].local.testBit(m);
    if(k==3 && l==1)return cb.src_joint[n].joint.testBit(m);
    return false;
  }
  void audit() {
    fpga::CongestionAuditIndex index(nets);size_t count=0;
    for(auto& t:device.tile_grid) {
      if(!t.cb_type)continue;
      std::ostringstream text;auto a=fpga::auditTileCongestion(t,text,index);
      require(!a.orphan_leases && !a.missing_leases && !a.missing_registrations && !a.stale_registrations && !a.conflicting_driver_nodes,text.str());++count;
    }
    std::cout<<"captured region audit tiles="<<count<<" bindings=8959 fragments="<<fragments<<'\n';
  }
};
}

void route_region_replay_regression() {
  const char* previous=std::getenv("SCALEPNR_PROFILE_REPAIR");
  struct RestoreProfile {
    bool present;std::string value;
    ~RestoreProfile() {if(present)setenv("SCALEPNR_PROFILE_REPAIR",value.c_str(),1);else unsetenv("SCALEPNR_PROFILE_REPAIR");}
  } restore{previous!=nullptr,previous?previous:""};
  setenv("SCALEPNR_PROFILE_REPAIR","1",1);
  Region f;f.audit();
  for(size_t i=1;i<f.witness.size();++i)require(f.edge(f.witness[i-1],f.witness[i]),"captured witness lost physical edge "+std::to_string(i));
  auto& router=f.tech.route;auto& b=f.task.net->routes[f.trigger_index];auto& route=b.owner->wires[b.route_index];
  require(route.size()==10 && fpga::hasRouteGaps(route),"captured failed route must retain its exact GAP");
  const std::vector<fpga::RouteCutNode> cuts{{{37,12},fpga::CB_NODE_SRC,64},{{37,14},fpga::CB_NODE_DST,64}};
  auto victims=fpga::findNetRouteCutVictims(cuts);
  require(victims.size()==1,"captured jump must belong to one real transit victim");
  auto victim=victims.front();const auto victim_binding=victim.net->routes[victim.binding_index];
  const bool victim_complete=fpga::isRouteComplete(victim_binding.owner->wires[victim_binding.route_index]);
  size_t busy_witness_nodes=0;
  for(const auto& node:f.witness) {
    auto[x,y,k,n]=node;
    busy_witness_nodes+=fpga::congestionNodeMask(f.tile({x,y}).cb,fpga::CBNodeNameType(k)).testBit(n);
  }
  require(busy_witness_nodes==3,"witness occupancy changed: one SRC, one DST, and its retained sink LOCAL");
  for(const auto& v:victims)require(fpga::canPreemptNetRouteFromNodes(*v.net,v.binding_index,cuts),"captured obstruction is not eligible transit");
  // The exhausted stage must not falsely report completion or mutate leases.
  router.route_stage_deadline_enabled=true;
  router.route_stage_deadline=std::chrono::steady_clock::now()-std::chrono::seconds(1);
  const auto revision=fpga::routeMutationRevision();
  fpga::resetRepairStatistics();
  require(!router.routeAtCurrentPlacement(f.task,131072,true,true),"expired stage reported successful repair");
  require(route.size()==10 && fpga::hasRouteGaps(route) && router.pending_route_todo.empty(),"deadline changed captured routes");
  require(fpga::routeMutationRevision()==revision && fpga::repairStatistics()[fpga::RepairCounter::source_expanded]==0,"expired stage mutated state or expanded search");
  router.route_stage_deadline_enabled=router.route_stage_deadline_expired=false;
  fpga::resetRepairStatistics();
  const bool complete=router.routeAtCurrentPlacement(f.task,131072,true,true);
  const auto work=fpga::repairStatistics();
  std::cout<<"captured region repair complete="<<complete<<" expanded="<<work[fpga::RepairCounter::source_expanded]
      <<" cuts="<<router.route_stats.preempt_success<<" pending="<<router.pending_route_todo.size()<<std::endl;
  require(complete && fpga::isRouteComplete(route),"fresh bounded repair missed the captured replacement");
  require(work[fpga::RepairCounter::source_expanded]>0 &&
      work[fpga::RepairCounter::source_expanded]<=131072 &&
      work[fpga::RepairCounter::source_success]==1,"captured repair exceeded its work bound");
  require(router.route_stats.preempt_success==1 &&
      router.route_stats.preempt_complete_victims==size_t(victim_complete) &&
      router.pending_route_todo.size()==1,"repair lost or duplicated displaced work");
  const auto& pending=router.pending_route_todo.front();
  require(pending.net==victim.net && pending.from==victim_binding.from &&
      pending.to==victim_binding.to && pending.from_port==victim_binding.from_port &&
      pending.to_port==victim_binding.to_port,"wrong displaced connection was queued");
  require(fpga::hasRouteGaps(victim_binding.owner->wires[victim_binding.route_index]),
      "displaced transit was not retained as an explicit partial route");
  for(const auto& cut:cuts) {
    const auto owners=fpga::findNetOwnersByNode(f.tile(cut.tile),cut.type,cut.node);
    require(owners.size()==1 && owners.front().net==f.task.net &&
        owners.front().binding_index==f.trigger_index,"replacement did not take the captured transit jump");
  }
  require(f.task.from->coord==C{55,6} && f.task.to->coord==C{38,14},"wire repair moved a captured endpoint");
  f.audit();
}
