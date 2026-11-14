#include "SourceInputFixture.h"
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
  else if(k==1 && l==1)cb.joint_joint[n].joint.setBit(m);
  else if(k==1 && l==0)cb.joint_local[n].local.setBit(m);
  else require(false,"unsupported captured arc");
}

}

// Example 062, chunk 6: Destination Moving timed out relocating a six-input
// LUT although its original input had a complete 39-node alternative. The
// only obstruction was (49,19)/SRC736 -> (50,18)/DST736, in a detached tail
// of an already-incomplete route. Two completed siblings do NOT use that jump.
// Keep all four captured route vectors, ownership flags, GAPs, resource/CB
// coordinates and the replacement path. Names are abstract numeric patterns.
// Unrelated chip congestion/inputs are omitted: this isolates whether repair
// is attempted BEFORE placement, not the cost of the full chip's frontier.
void moving_partial_routing_regression(int mode) {
  SourceInputFixture f(false,0,32,90,40);
  std::array<std::vector<W>,4> routes;
  std::vector<N> witness;
  std::ifstream file(std::filesystem::path(__FILE__).parent_path()/"moving_partial_routing_snapshot.txt");
  require(bool(file),"missing captured partial-routing snapshot");
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
      w.type=W::Type(type);require(r>=0 && r<4,"bad captured route index");routes[r].push_back(w);
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
  // Preserve actual site/slot identities and the resource/CB split.
  const std::array<C,5> resources{{{61,5},{59,4},{55,16},{51,20},{33,2}}};
  const std::array<C,5> cbs{{{62,5},{58,4},{54,16},{50,20},{32,2}}};
  const std::array<int,5> positions{{131,135,15,131,143}};
  for(size_t i=0;i<resources.size();++i)slots(f,resources[i],cbs[i],positions[i]/128*4+(positions[i]%128)/4);
  slots(f,{51,18},{50,18},1);
  auto output=[&](C r,int pos,int local) {
    auto& t=f.tile(r);auto& cb=f.tile(t.cb_coord);
    t.tile_type->pin_map.output_nodes[t.getNodeNum("LUT6","O",pos)+(pos/128)*256].setBit(local);
    cb.cb_type->local_output_nodes.setBit(local);
  };
  auto input=[&](C r,int pos,int local) {
    auto& t=f.tile(r);t.tile_type->pin_map.nodes[{"LUT6","I0",pos}].setBit(local);
    f.tile(t.cb_coord).cb_type->local_input_nodes.setBit(local);
  };
  output(resources[0],positions[0],routes[0].front().local);
  output(resources[4],positions[4],routes[3].front().local);
  for(int i=0;i<3;++i)input(resources[i+1],positions[i+1],routes[i].back().local);
  input({51,18},7,104);
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  auto* victim_source=f.cell("FILL_DRIVER_101","LUT6",{{"O",true}},resources[0],positions[0]);
  auto* source=f.cell("FILL_DRIVER_103","LUT6",{{"O",true}},resources[4],positions[4]);
  std::array<rtl::Inst*,4> sinks;
  for(int i=0;i<3;++i)sinks[i]=f.cell("FILL_LOAD_"+std::to_string(107+i),"LUT6",{{"I0",false}},resources[i+1],positions[i+1]);
  sinks[3]=f.cell("FILL_LOAD_113","LUT6",{{"I0",false}},{51,18},7);
  for(auto* inst:{victim_source,source,sinks[0],sinks[1],sinks[2],sinks[3]})inst->outline.fixed=inst->pnr_completed=true;
  auto* victim_net=f.connect(*victim_source,"O",*sinks[0],"I0");
  for(int i=1;i<3;++i) {
    auto& p=SourceInputFixture::port(*sinks[i],"I0");
    p.port_ref->designator=SourceInputFixture::port(*victim_source,"O").port_ref->designator;
    p.set(&rtl::Conn::fromBase(SourceInputFixture::port(*victim_source,"O")));
  }
  auto* request_net=f.connect(*source,"O",*sinks[3],"I0");
  // Restore captured partial route storage rather than pretending its GAPs
  // can be committed as connected wires. Lease only nodes with real owners;
  // auditAll verifies every registration, lease and physical driver afterward.
  for(int i=0;i<4;++i) {
    sinks[i]->wires.push_back(routes[i]);auto* net=i==3?request_net:victim_net;
    auto id=fpga::attachNetRoute(*net,*sinks[i],0,i==3?source:victim_source,sinks[i],"O","I0",
        SourceInputFixture::port(*sinks[i],"I0").makeNetName());
    fpga::registerNetRouteTiles(*net,sinks[i]->wires.front(),id);
  }
  for(auto[x,y,k,n]:nodes) {
    auto& t=f.tile({x,y});if(fpga::findNetOwnersByNode(t,fpga::CBNodeNameType(k),n).empty())continue;
    if(k==0)t.cb.local.local.setBit(n);
    else if(k==1)t.cb.joint.jump.setBit(n);
    else if(k==3)t.cb.src.jump.setBit(n);
    else if(k==4)t.cb.dst.jump.setBit(n);
  }
  f.auditAll();
  require(witness.size()==39 && routes[0].size()==105 && routes[1].size()==13 && routes[2].size()==11 && routes[3].size()==20,"captured geometry changed");
  require(fpga::isRouteComplete(sinks[0]->wires[0]) && fpga::isRouteComplete(sinks[2]->wires[0]) &&
      fpga::hasRouteGaps(sinks[1]->wires[0]) && fpga::hasRouteGaps(sinks[3]->wires[0]),"incorrect captured completion state");
  const std::vector<fpga::RouteCutNode> cuts{{{49,19},fpga::CB_NODE_SRC,736},{{50,18},fpga::CB_NODE_DST,736}};
  auto victims=fpga::findNetRouteCutVictims(cuts);
  require(victims.size()==1 && victims[0].net==victim_net && victims[0].binding_index==1,"captured jump must affect only the incomplete sibling");
  auto& router=f.tech.route;router.moving_stage=router.complete_routes_only=router.preserve_routes=true;
  router.moving_sources_stage=false;router.route_deadends_enabled=false;router.route_preemption_enabled=true;
  router.route_recursion_budget=router.iteration_limit=router.route_suffix_depth_limit=200;
  router.indexSourceRoute(victim_net,victim_source,"O");router.indexSourceRoute(request_net,source,"O");
  pnr::RouteDesign::RouteTask task{source,sinks[3],request_net,"O","I0",SourceInputFixture::port(*sinks[3],"I0").makeNetName()};
  task.fanout=true;task.endpoints_prepared=true;
  require(!router.routeAtCurrentPlacement(task,32768,true),"snapshot unexpectedly has a free completion");
  bool wire_only=false;std::string reason;std::vector<pnr::RouteDesign::RouteTask> affected;
  bool done=mode==1 ? router.routeAtCurrentPlacement(task,32768,true,true)
      : router.moveUnfinishedCell(task,&affected,&task,&reason,false,&wire_only);
  std::cout<<"partial-current done="<<done<<" wire_only="<<wire_only<<" placement_scans="<<router.move_tried_placements.size()
      <<" cuts="<<router.route_stats.preempt_success<<" pending="<<router.pending_route_todo.size()<<" reason="<<reason<<'\n';
  require(done && (mode==1 || wire_only) && router.move_tried_placements.empty(),"safe incomplete-only routing was deferred until relocation");
  // Completion installs a new route binding before retiring the old slot;
  // inspect that live binding, while the incomplete victim retains its gap.
  require(router.routeTaskIsComplete(task) && fpga::hasRouteGaps(sinks[1]->wires[0]),"request/victim completion incorrect");
  require(router.route_stats.preempt_success==1 && !router.route_stats.preempt_complete_victims &&
      router.pending_route_todo.size()==1 &&
      router.pending_route_todo[0].to==sinks[1],"must enqueue only the already-incomplete victim once");
  require(affected.empty() && source->coord==C{33,2} && sinks[3]->coord==C{51,18},"wire-only repair changed placement");
  for(int i:{0,2}) {
    const auto& current=sinks[i]->wires[0];require(fpga::isRouteComplete(current) && current.size()==routes[i].size(),"completed sibling was damaged");
    for(size_t j=0;j<current.size();++j) {
      const auto& a=current[j];const auto& b=routes[i][j];
      require(a.type==b.type && a.from==b.from && a.to==b.to && a.local==b.local && a.jump==b.jump && a.dst==b.dst &&
          a.pos==b.pos && a.route_jump==b.route_jump && a.joint==b.joint && a.joint2==b.joint2 &&
          a.from_node_type==b.from_node_type && a.from_node==b.from_node && a.to_node_type==b.to_node_type && a.to_node==b.to_node &&
          a.resource==b.resource && a.resource_node==b.resource_node && a.pin_dir==b.pin_dir &&
          a.shared==b.shared && a.owns_dst==b.owns_dst && a.owns_landing==b.owns_landing,"completed sibling changed");
    }
  }
  f.auditAll();
}
