#include "SourceInputFixture.h"
#include "RepairStatistics.h"
#include <filesystem>
#include <fstream>
#include <set>
#include <tuple>
#include <iostream>
using namespace routing_tests;
namespace {
using C=fpga::Coord;
using W=fpga::Wire;
using N=std::tuple<int,int,int,int>;
void init(SourceInputFixture& f,C at) {
  auto& t=f.tile(at);if(t.cb_type)return;
  auto& cb=f.device.cb_types.emplace_back();cb.type_id=cb.base_type_id=f.device.cb_types.size()-1;
  cb.name="FILL_PARTIAL_MATRIX_"+std::to_string(cb.type_id);
  auto& tt=f.device.tile_types.emplace_back();tt.name="FILL_PARTIAL_RESOURCE_"+std::to_string(cb.type_id);
  tt.type=t.type=fpga::Tile::TILE_LUTS;t.tile_type=&tt;t.cb_type=t.cb.type=&cb;
}
void slots(SourceInputFixture& f,C at,C cb_at,int bit) {
  init(f,at);init(f,cb_at);auto& t=f.tile(at);t.cb_coord=cb_at;
  if(at!=cb_at)f.tile(cb_at).attached_resource_tiles.push_back(&t);
  for(auto type:{fpga::ELEMENT_LUT5,fpga::ELEMENT_LUT1,fpga::ELEMENT_MUXF7}) {
    fpga::Element e;e.name="FILL_SLOT_"+std::to_string(type);e.type=type;e.bitmap_pos=bit;
    e.elements_to_left=int(type);
    t.tile_type->elements.push_back(e);
  }
  for(int site=0;site<2;++site)t.tile_type->sites.push_back(
      fpga::SiteModel{.name="FILL_SITE_"+std::to_string(site),.type="LOGIC",.pos=site});
}
void arc(SourceInputFixture& f,N a,N b) {
  auto[x,y,k,n]=a;auto[u,v,l,m]=b;init(f,{x,y});init(f,{u,v});
  SourceInputFixture::name(f.tile({x,y}),fpga::CBNodeNameType(k),n);
  SourceInputFixture::name(f.tile({u,v}),fpga::CBNodeNameType(l),m);
  auto& cb=*f.tile({x,y}).cb_type;
  if(k==3 && l==4) {
    fpga::CBType::ResolvedJump j;j.delta={u-x,v-y};j.target_tile_coord=true;
    j.target_cb_type_id=f.tile({u,v}).cb_type->type_id;j.dsts.jump.setBit(m);
    auto& links=cb.dst_by_src[n];
    if(links.empty())links.push_back(j);
    else require(links.front().delta==j.delta && links.front().dsts.jump==j.dsts.jump,"conflicting captured jump");
    return;
  }
  require(x==u && y==v,"unexpected non-jump captured arc");
  if(k==0 && l==3)cb.local_src[n].jump.setBit(m);
  else if(k==0 && l==1)cb.local_joint[n].joint.setBit(m);
  else if(k==0 && l==0)cb.local_local[n].local.setBit(m);
  else if(k==4 && l==3)cb.dst_src[n].jump.setBit(m);
  else if(k==4 && l==1)cb.dst_joint[n].joint.setBit(m);
  else if(k==4 && l==0)cb.dst_local[n].local.setBit(m);
  else if(k==3 && l==1)cb.src_joint[n].joint.setBit(m);
  else if(k==1 && l==1)cb.joint_joint[n].joint.setBit(m);
  else if(k==1 && l==0)cb.joint_local[n].local.setBit(m);
  else require(false,"unsupported captured arc");
}

}

// Example 064 exhausted Destination Moving while 854 tasks remained. In that
// snapshot this retained shared prefix needed only three free jumps to close
// its terminal GAP. Preserve the six numeric route vectors and their actual
// owners. Unrelated chip congestion and the other LUT inputs are omitted.
// Two synthetic free trees add breadth at the endpoints: they expose the
// cost of restarting from the source instead of repairing the existing GAP.
// The witness runs the existing production gap repair independently; the
// main test exercises Destination Moving's production entry point.
void moving_local_gap_regression(bool witness_only) {
  const char* previous=std::getenv("SCALEPNR_PROFILE_REPAIR");
  struct Restore { bool present; std::string value; ~Restore() {
    if(present)setenv("SCALEPNR_PROFILE_REPAIR",value.c_str(),1);
    else unsetenv("SCALEPNR_PROFILE_REPAIR");
  }} restore{previous!=nullptr,previous?previous:""};
  setenv("SCALEPNR_PROFILE_REPAIR","1",1);
  SourceInputFixture f(false,0,16,180,40);
  std::array<std::vector<W>,6> routes;
  std::vector<N> witness;
  std::ifstream file(std::filesystem::path(__FILE__).parent_path()/"moving_local_gap_snapshot.txt");
  require(bool(file),"missing captured partial-recovery snapshot");
  for(std::string line;std::getline(file,line);) {
    if(line.empty() || line[0]=='#')continue;
    std::istringstream in(line);char tag;in>>tag;
    if(tag=='N') {int x,y,k,n;in>>x>>y>>k>>n;witness.emplace_back(x,y,k,n);}
    else {
      require(tag=='W',"unknown captured record");int r,type;W w;
      in>>r>>w.from.x>>w.from.y>>w.to.x>>w.to.y>>w.resource.x>>w.resource.y
        >>type>>w.local>>w.pos>>w.jump>>w.route_jump>>w.dst>>w.joint>>w.joint2
        >>w.from_node_type>>w.from_node>>w.to_node_type>>w.to_node
        >>w.resource_node>>w.pin_dir>>w.shared>>w.owns_dst>>w.owns_landing;
      w.type=W::Type(type);require(r>=0 && r<6,"bad captured route index");routes[r].push_back(w);
    }
    require(bool(in),"incomplete captured record");
  }
  std::set<N> nodes;
  auto node=[&](C c,int k,int n) {
    if(n<0)return;
    init(f,c);SourceInputFixture::name(f.tile(c),fpga::CBNodeNameType(k),n);
    nodes.emplace(c.x,c.y,k,n);
  };
  auto edge=[&](C c,int k,int n,C to,int l,int m) {
    node(c,k,n);node(to,l,m);arc(f,{c.x,c.y,k,n},{to.x,to.y,l,m});
  };
  for(const auto& route:routes)for(const auto& w:route) {
    init(f,w.from);init(f,w.to);
    if(w.type==W::WIRE_GAP)continue; // a gap is NOT a physical bypass
    if(w.type==W::WIRE_TILE_PIN) {node(w.from,0,w.local);continue;}
    if(w.type==W::WIRE_ROUTE_EDGE) {
      edge(w.from,w.from_node_type,w.from_node,w.to,w.to_node_type,w.to_node);continue;
    }
    int kind=w.pos==0?0:4, n=w.local;
    for(int j:{w.joint2,w.joint})if(j>=0) {edge(w.from,kind,n,w.from,1,j);kind=1;n=j;}
    if(w.jump>=0) {
      edge(w.from,kind,n,w.from,3,w.jump);edge(w.from,3,w.jump,w.to,4,w.dst);
    } else edge(w.from,kind,n,w.from,0,route.back().local);
  }
  for(size_t i=1;i<witness.size();++i) {
    auto[x,y,k,n]=witness[i-1];auto[u,v,l,m]=witness[i];
    edge({x,y},k,n,{u,v},l,m);
  }

  constexpr int count=(1<<16)-1, first_leaf=(count-1)/2;
  auto at=[](int n,int y) {return C{110+n/CB_MAX_NODES,y};};
  auto id=[](int n) {return n%CB_MAX_NODES;};
  for(int y:{30,32})for(int n=0;n<count;n+=CB_MAX_NODES)init(f,at(n,y));
  f.resolvedLocal({45,7},149,at(0,30),fpga::CB_NODE_LOCAL,0);
  f.resolvedLocal(at(0,32),0,{36,12},fpga::CB_NODE_LOCAL,109);
  for(int n=0;n<first_leaf;++n)for(int child:{2*n+1,2*n+2}) {
    f.resolvedLocal(at(n,30),id(n),at(child,30),child>=first_leaf?fpga::CB_NODE_SRC:fpga::CB_NODE_LOCAL,id(child));
    f.resolvedLocal(at(child,32),id(child),at(n,32),fpga::CB_NODE_LOCAL,id(n));
  }
  const C resource{46,7}, source_cb{45,7}; const int source_pos=131;
  slots(f,resource,source_cb,4);
  auto& source_tile=f.tile(resource);
  source_tile.tile_type->pin_map.output_nodes[source_tile.getNodeNum("LUT6","O",source_pos)+256].setBit(149);
  f.tile(source_cb).cb_type->local_output_nodes.setBit(149);
  const std::array<const char*,6> ports{{"I0","I0","I0","I0","I1","I1"}};
  std::array<rtl::Inst*,6> sinks;
  for(int i=0;i<6;++i) {
    const W& pin=routes[i].back(); const int pos=pin.pos;
    slots(f,pin.resource,pin.from,pos/128*4+(pos%128)/4);
    auto& t=f.tile(pin.resource);
    t.tile_type->pin_map.nodes[{"LUT6",ports[i],pos}].setBit(pin.local);
    f.tile(pin.from).cb_type->local_input_nodes.setBit(pin.local);
  }
  auto* source=f.cell("FILL_GAP_DRIVER_401","LUT6",{{"O",true}},resource,source_pos);
  for(int i=0;i<6;++i) {
    const W& pin=routes[i].back(); const int pos=pin.pos;
    sinks[i]=f.cell("FILL_GAP_LOAD_"+std::to_string(409+i),"LUT6",{{ports[i],false}},pin.resource,pos);
    sinks[i]->outline.fixed=sinks[i]->pnr_completed=true;
  }
  source->outline.fixed=source->pnr_completed=true;
  auto* net=f.connect(*source,"O",*sinks[0],ports[0]);
  for(int i=1;i<6;++i) {
    auto& p=SourceInputFixture::port(*sinks[i],ports[i]);
    p.port_ref->designator=SourceInputFixture::port(*source,"O").port_ref->designator;
    p.set(&rtl::Conn::fromBase(SourceInputFixture::port(*source,"O")));
  }
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  for(int i=0;i<6;++i) {
    sinks[i]->wires.push_back(routes[i]);
    auto bi=fpga::attachNetRoute(*net,*sinks[i],0,source,sinks[i],"O",ports[i],SourceInputFixture::port(*sinks[i],ports[i]).makeNetName());
    fpga::registerNetRouteTiles(*net,sinks[i]->wires[0],bi);
  }
  for(auto[x,y,k,n]:nodes) {
    auto& t=f.tile({x,y});if(fpga::findNetOwnersByNode(t,fpga::CBNodeNameType(k),n).empty())continue;
    if(k==0)t.cb.local.local.setBit(n);
    else if(k==1)t.cb.joint.jump.setBit(n);
    else if(k==3)t.cb.src.jump.setBit(n);
    else if(k==4)t.cb.dst.jump.setBit(n);
  }
  f.auditAll();
  require(routes[5].size()==6 && fpga::hasRouteGaps(routes[5]) && witness.size()==8,"captured gap geometry changed");
  for(int i=0;i<5;++i)require(fpga::isRouteComplete(routes[i]),"shared-prefix owner must be complete");
  auto& router=f.tech.route;router.moving_stage=router.complete_routes_only=router.preserve_routes=true;
  router.moving_sources_stage=false;router.route_deadends_enabled=false;router.route_preemption_enabled=true;
  router.route_recursion_budget=router.iteration_limit=router.route_suffix_depth_limit=200;
  router.indexSourceRoute(net,source,"O");
  pnr::RouteDesign::RouteTask task{source,sinks[5],net,"O",ports[5],SourceInputFixture::port(*sinks[5],ports[5]).makeNetName()};
  task.fanout=true;task.endpoints_prepared=true;
  bool wire_only=false;std::string reason;std::vector<pnr::RouteDesign::RouteTask> affected;
  fpga::resetRepairStatistics();
  const bool done=witness_only ? router.recoverCurrentPosition(task,32768,false,true,nullptr,false)
      : router.moveUnfinishedCell(task,&affected,&task,&reason,false,&wire_only);
  const auto work=fpga::repairStatistics();
  std::cout<<"local-gap done="<<done<<" wire_only="<<wire_only<<" gap_expanded="<<work[fpga::RepairCounter::gap_expanded]
      <<" source_expanded="<<work[fpga::RepairCounter::source_expanded]<<" cuts="<<router.route_stats.preempt_success<<" reason="<<reason<<'\n';
  require(done && (witness_only || wire_only) && fpga::isRouteComplete(sinks[5]->wires[0]),"local gap did not complete");
  require(work[fpga::RepairCounter::source_expanded]+work[fpga::RepairCounter::gap_expanded]<=16,
      "Moving flooded the source despite the short free retained-gap repair");
  require(router.move_tried_placements.empty() && affected.empty() && router.pending_route_todo.empty() && !router.route_stats.preempt_success,
      "free local repair moved a cell or preempted a connection");
  for(int i=0;i<6;++i) {
    const auto& live=sinks[i]->wires[0];const size_t preserved=i==5?4:routes[i].size();
    require(fpga::isRouteComplete(live) && (i==5 || live.size()==preserved),"completed owner changed");
    for(size_t j=0;j<preserved;++j) {
      const auto& a=live[j];const auto& b=routes[i][j];
      require(a.type==b.type && a.from==b.from && a.to==b.to && a.local==b.local && a.jump==b.jump && a.dst==b.dst &&
          a.pos==b.pos && a.route_jump==b.route_jump && a.joint==b.joint && a.joint2==b.joint2 &&
          a.from_node_type==b.from_node_type && a.from_node==b.from_node && a.to_node_type==b.to_node_type && a.to_node==b.to_node &&
          a.resource==b.resource && a.resource_node==b.resource_node && a.pin_dir==b.pin_dir &&
          a.shared==b.shared && a.owns_dst==b.owns_dst && a.owns_landing==b.owns_landing,"retained prefix or completed owner was rewritten");
    }
    require(sinks[i]->coord==routes[i].back().resource && sinks[i]->pos==routes[i].back().pos,"sink moved");
  }
  require(source->coord==resource && source->pos==source_pos,"driver moved");
  f.auditAll();
}
