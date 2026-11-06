#include "SourceInputFixture.h"
#include <array>
#include <iostream>
using namespace routing_tests;
namespace {
using Coord=fpga::Coord;
using Wire=fpga::Wire;
using Route=std::vector<Wire>;
Wire fragment(int type,Coord from,Coord to,int local,int pos,int src,int dst,
    int joint,int joint2,int from_kind,int from_node,int to_kind,int to_node,
    bool owns_dst,bool owns_landing) {
  Wire w;w.type=static_cast<Wire::Type>(type);w.from=from;w.to=to;
  w.local=local;w.pos=pos;w.jump=w.route_jump=src;w.dst=dst;
  w.joint=joint;w.joint2=joint2;w.from_node_type=from_kind;w.from_node=from_node;
  w.to_node_type=to_kind;w.to_node=to_node;w.owns_dst=owns_dst;w.owns_landing=owns_landing;
  return w;
}
void matrix(SourceInputFixture& f,Coord at,int banks=0) {
  auto& tile=f.tile(at);
  if(!tile.cb_type) {
    auto& cb=f.device.cb_types.emplace_back();cb.name="cycle_matrix_"+std::to_string(at.x)+"_"+std::to_string(at.y);
    cb.type_id=cb.base_type_id=int(f.device.cb_types.size()-1);
    auto& type=f.device.tile_types.emplace_back();type.name="cycle_resource_"+std::to_string(at.x)+"_"+std::to_string(at.y);
    type.type=tile.type=fpga::Tile::TILE_LUTS;tile.tile_type=&type;tile.cb_type=tile.cb.type=&cb;
    tile.coord=tile.cb_coord=at;tile.name=at;tile.full_name=type.name;
  }
  for(int bank=0;bank<banks;++bank)for(auto kind:{fpga::ELEMENT_LUT5,fpga::ELEMENT_LUT1}) {
    fpga::Element e;e.name="cycle_slot_"+std::to_string(bank)+"_"+std::to_string(kind);
    e.type=kind;e.bitmap_pos=bank;tile.tile_type->elements.push_back(e);
  }
}
void topology(SourceInputFixture& f,const Route& route) {
  for(const auto& w:route) {
    matrix(f,w.from);matrix(f,w.to);
    auto name=[&](Coord at,fpga::CBNodeNameType kind,int node) {
      if(node>=0)SourceInputFixture::name(f.tile(at),kind,node);
    };
    if(w.type==Wire::WIRE_TILE_PIN)name(w.from,fpga::CB_NODE_LOCAL,w.local);
    if(w.type==Wire::WIRE_CROSSBAR) {
      name(w.from,w.pos==0?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST,w.local);
      name(w.from,fpga::CB_NODE_SRC,w.jump);name(w.to,fpga::CB_NODE_DST,w.dst);
      name(w.from,fpga::CB_NODE_JOINT,w.joint);name(w.from,fpga::CB_NODE_JOINT,w.joint2);
    }
    if(w.type==Wire::WIRE_ROUTE_EDGE) {
      name(w.from,static_cast<fpga::CBNodeNameType>(w.from_node_type),w.from_node);
      name(w.to,static_cast<fpga::CBNodeNameType>(w.to_node_type),w.to_node);
    }
    if(w.type==Wire::WIRE_CROSSBAR) {
      if(w.jump>=0)f.edge(w.from,w.to,w.local,w.jump,w.dst,w.pos==0);
      else {
        auto& cb=*f.tile(w.from).cb_type;
        if(w.joint<0)cb.dst_local[w.local].local.setBit(route.back().local);
        else {
          cb.dst_joint[w.local].joint.setBit(w.joint2<0?w.joint:w.joint2);
          if(w.joint2>=0)cb.joint_joint[w.joint2].joint.setBit(w.joint);
          cb.joint_local[w.joint].local.setBit(route.back().local);
        }
      }
    } else if(w.type==Wire::WIRE_ROUTE_EDGE) {
      auto& cb=*f.tile(w.from).cb_type;
      if(w.from_node_type==fpga::CB_NODE_DST && w.to_node_type==fpga::CB_NODE_LOCAL)
        cb.dst_local[w.from_node].local.setBit(w.to_node);
      else if(w.from_node_type==fpga::CB_NODE_LOCAL && w.to_node_type==fpga::CB_NODE_LOCAL && w.from==w.to)
        cb.local_local[w.from_node].local.setBit(w.to_node);
      else if(w.from_node_type==fpga::CB_NODE_LOCAL && w.to_node_type==fpga::CB_NODE_SRC)
        cb.local_src[w.from_node].jump.setBit(w.to_node);
      else if(w.from_node_type==fpga::CB_NODE_SRC && w.to_node_type==fpga::CB_NODE_DST) {
        auto& targets=cb.dst_by_src[w.from_node];
        fpga::CBType::ResolvedJump jump{};jump.delta=w.to-w.from;jump.target_tile_coord=true;
        jump.target_cb_type_id=f.tile(w.to).cb_type->type_id;jump.dsts.jump.setBit(w.to_node);
        require(targets.empty(),"duplicate recorded explicit jump");targets.push_back(jump);
      } else require(false,"unsupported recorded cycle edge");
    }
  }
}
void install(SourceInputFixture& f,rtl::Net& net,rtl::Inst& from,rtl::Inst& to,Route route) {
  route.front().pin_dir=fpga::TILE_PIN_OUTPUT;route.back().pin_dir=fpga::TILE_PIN_INPUT;
  size_t failed=0;
  const auto first_private=std::find_if(route.begin(),route.end(),[](const auto& w){return !w.shared;});
  Route leases(first_private,route.end());
  const bool installed=pnr::commitPreparedRoute(leases,&failed);
  require(installed,"cannot install captured cycle route to "+to.makeName()+" fragment="+std::to_string(failed));
  to.wires.push_back(std::move(route));
  auto id=fpga::attachNetRoute(net,to,to.wires.size()-1,&from,&to,"O","I0",SourceInputFixture::port(to,"I0").makeNetName());
  fpga::registerNetRouteTiles(net,to.wires.back(),id);
}
}
// Example 1 run 7: FOUR connections (three physical drivers) were repaired
// 1526 times each. The first repair cuts two branches and a third driver's
// transit; those repairs displace the first connection again. Preserve the
// actual prefixes, LOCAL bridges, shared branches and contested jumps. Names
// are synthetic patterns and every occupied node has a real route owner.
// A separate free-only placement witness establishes joint feasibility.
void moving_preemption_cycle_regression(bool wire_bypass,bool fallback) {
  for(bool reference:{true,false}) {
  SourceInputFixture f(false,0,32,60,24);
  Route first{
    fragment(1,{54,17},{54,17},155,11,-1,-1,-1,-1,-1,-1,-1,-1,true,false),
    fragment(0,{54,17},{53,18},155,0,3618,3618,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{53,18},{44,18},3618,1,3075,3080,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{44,18},{41,17},3080,1,3811,3811,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{41,17},{41,11},3811,1,163,163,-1,-1,-1,-1,-1,-1,true,true),
    fragment(2,{41,11},{41,11},-1,-1,-1,-1,-1,-1,4,163,0,184,true,false),
    fragment(2,{41,11},{41,11},-1,-1,-1,-1,-1,-1,0,184,3,3378,true,false),
    fragment(2,{41,11},{37,15},-1,-1,-1,-1,-1,-1,3,3378,4,3378,true,false),
    fragment(0,{37,15},{32,11},3378,1,3539,3539,-1,-1,-1,-1,-1,-1,true,true),
    fragment(2,{32,11},{32,11},-1,-1,-1,-1,-1,-1,4,3539,0,184,true,false),
    fragment(2,{32,11},{32,11},-1,-1,-1,-1,-1,-1,0,184,3,1029,true,false),
    fragment(2,{32,11},{41,11},-1,-1,-1,-1,-1,-1,3,1029,4,1030,true,false),
    fragment(0,{41,11},{44,11},1030,1,259,257,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{44,11},{45,11},257,1,258,262,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{45,11},{45,11},262,1,-1,-1,-1,-1,-1,-1,-1,-1,true,false),
    fragment(1,{45,11},{45,11},101,139,-1,-1,-1,-1,-1,-1,-1,-1,true,false),
  };
  Route branch{
    fragment(1,{40,11},{40,11},156,15,-1,-1,-1,-1,-1,-1,-1,-1,true,false),
    fragment(0,{40,11},{41,11},156,0,261,258,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{41,11},{44,11},258,1,259,257,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{44,11},{44,12},257,1,17,20,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{44,12},{44,12},20,1,-1,-1,36,51,-1,-1,-1,-1,true,false),
    fragment(1,{44,12},{44,12},112,11,-1,-1,-1,-1,-1,-1,-1,-1,true,false),
  };
  Route second_branch{
    fragment(1,{40,11},{40,11},156,15,-1,-1,-1,-1,-1,-1,-1,-1,true,false),
    fragment(0,{40,11},{41,11},156,0,261,258,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{41,11},{44,11},258,1,259,257,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{44,11},{49,11},257,1,1026,1025,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{49,11},{44,15},1025,1,3377,3377,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{44,15},{44,16},3377,1,17,20,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{44,16},{44,16},20,1,-1,-1,36,51,-1,-1,-1,-1,true,false),
    fragment(1,{44,16},{44,16},112,11,-1,-1,-1,-1,-1,-1,-1,-1,true,false),
  };
  Route third{
    fragment(1,{40,11},{40,11},152,143,-1,-1,-1,-1,-1,-1,-1,-1,true,false),
    fragment(0,{40,11},{44,7},152,0,979,979,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{44,7},{49,7},979,1,1030,1027,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{49,7},{57,7},1027,1,1031,1031,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{57,7},{53,11},1031,1,3379,3379,-1,-1,-1,-1,-1,-1,true,true),
    fragment(2,{53,11},{53,11},-1,-1,-1,-1,-1,-1,4,3379,0,140,true,false),
    fragment(2,{53,11},{53,11},-1,-1,-1,-1,-1,-1,0,140,0,177,true,false),
    fragment(2,{53,11},{53,11},-1,-1,-1,-1,-1,-1,0,177,3,3078,true,false),
    fragment(2,{53,11},{44,11},-1,-1,-1,-1,-1,-1,3,3078,4,3079,true,false),
    fragment(0,{44,11},{45,11},3079,1,258,262,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{45,11},{49,11},262,1,259,257,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{49,11},{50,10},257,1,737,737,-1,-1,-1,-1,-1,-1,true,true),
    fragment(0,{50,10},{50,10},737,1,-1,-1,-1,-1,-1,-1,-1,-1,true,false),
    fragment(1,{50,10},{50,10},91,3,-1,-1,-1,-1,-1,-1,-1,-1,true,false),
  };
  for(const auto& route:{first,branch,second_branch,third})topology(f,route);
  matrix(f,{54,17},1);matrix(f,{40,11},2);matrix(f,{45,11},1);
  matrix(f,{44,12},1);matrix(f,{44,16},1);matrix(f,{50,10},1);matrix(f,{53,18},1);
  auto output=[&](Coord at,int pos,int pin) {
    auto& t=f.tile(at);t.tile_type->pin_map.output_nodes[t.getNodeNum("LUT6","O",pos)].setBit(pin);
    t.cb_type->local_output_nodes.setBit(pin);
  };
  auto input=[&](Coord at,int pin) {
    auto& t=f.tile(at);t.tile_type->pin_map.nodes[{"LUT6","I0",3}].setBit(pin);
    t.cb_type->local_input_nodes.setBit(pin);
  };
  output({54,17},3,155);output({40,11},3,156);output({40,11},7,152);
  input({45,11},101);input({44,12},112);input({44,16},112);input({50,10},91);input({53,18},87);
  f.terminal({53,18},3618,87);
  if(wire_bypass) {
    f.edge({41,11},{45,11},1030,2001,2002);f.terminal({45,11},2002,101);
  }
  for(auto& cb:f.device.cb_types)cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  auto* a=f.cell("cycle_driver_101","LUT6",{{"O",true}},{54,17},3);
  auto* b=f.cell("cycle_driver_103","LUT6",{{"O",true}},{40,11},3);
  auto* c=f.cell("cycle_driver_107","LUT6",{{"O",true}},{40,11},7);
  auto* x=f.cell("cycle_sink_109","LUT6",{{"I0",false}},{45,11},3);
  auto* y=f.cell("cycle_sink_113","LUT6",{{"I0",false}},{44,12},3);
  auto* z=f.cell("cycle_sink_127","LUT6",{{"I0",false}},{44,16},3);
  auto* u=f.cell("cycle_sink_131","LUT6",{{"I0",false}},{50,10},3);
  for(auto* i:{a,b,c,x,y,z,u})i->outline.fixed=i->pnr_completed=true;
  auto* an=f.connect(*a,"O",*x,"I0");auto* bn=f.connect(*b,"O",*y,"I0");auto* cn=f.connect(*c,"O",*u,"I0");
  auto& zi=SourceInputFixture::port(*z,"I0");zi.port_ref->designator=SourceInputFixture::port(*b,"O").port_ref->designator;
  zi.set(&rtl::Conn::fromBase(SourceInputFixture::port(*b,"O")));
  install(f,*an,*a,*x,std::move(first));
  const std::vector<fpga::RouteCutNode> cut{{{41,11},fpga::CB_NODE_SRC,259},{{44,11},fpga::CB_NODE_SRC,258}};
  std::cout<<"cycle initial fragments="<<x->wires.front().size()<<" complete="<<fpga::isRouteComplete(x->wires.front())<<" victims="<<fpga::findNetRouteCutVictims(cut).size()<<" preemptible="<<fpga::canPreemptNetRouteFromNodes(*an,0,cut)<<std::endl;
  require(fpga::unrouteNetRoutesFromNodes(fpga::findNetRouteCutVictims(cut),cut,true),"cannot open the captured first-route gap");
  require(fpga::hasRouteGaps(x->wires.front()),"reference preemption did not leave a real gap");
  install(f,*bn,*b,*y,std::move(branch));
  for(size_t i=0;i<3;++i){second_branch[i].shared=true;second_branch[i].owns_landing=false;}
  second_branch[3].owns_dst=false;second_branch[3].pos=2;
  install(f,*bn,*b,*z,std::move(second_branch));install(f,*cn,*c,*u,std::move(third));
  f.auditAll();
  auto& router=f.tech.route;router.moving_stage=router.complete_routes_only=router.preserve_routes=true;
  router.moving_sources_stage=false;router.route_deadends_enabled=false;router.route_preemption_enabled=!reference;
  router.route_recursion_budget=router.iteration_limit=router.route_suffix_depth_limit=200;
  router.indexSourceRoute(an,a,"O");router.indexSourceRoute(bn,b,"O");router.indexSourceRoute(cn,c,"O");
  auto make_task=[](rtl::Net* net,rtl::Inst* from,rtl::Inst* to) {
    pnr::RouteDesign::RouteTask t{from,to,net,"O","I0",SourceInputFixture::port(*to,"I0").makeNetName()};
    t.fanout=true;t.endpoints_prepared=true;return t;
  };
  std::array<pnr::RouteDesign::RouteTask,4> tasks{make_task(an,a,x),make_task(bn,b,y),make_task(bn,b,z),make_task(cn,c,u)};
  auto complete=[&] {for(auto* i:{x,y,z,u})if(i->wires.empty() || !fpga::isRouteComplete(i->wires.front()))return false;return true;};
  if(fallback && !reference) {
    require(router.recoverCurrentPosition(tasks[0],32768,true,true),"explicit final fallback lost legal transit preemption");
    require(fpga::isRouteComplete(x->wires.front()) && router.pending_route_todo.size()==3 &&
        router.route_stats.preempt_success==3,"fallback did not queue all three real victims exactly once");
    require(!fpga::isRouteComplete(y->wires.front()) && !fpga::isRouteComplete(z->wires.front()) &&
        !fpga::isRouteComplete(u->wires.front()),"fallback did not preserve the captured dependency pattern");
    require(router.route_preemption_enabled && !router.moving_focus_inst,"in-place fallback leaked its scope");
    f.auditAll();continue;
  }
  size_t repairs=0,moves=0;bool first_probe=false;
  for(size_t round=0;round<3 && !complete();++round)for(auto& task:tasks) {
    if(fpga::isRouteComplete(task.to->wires.front()))continue;
    if (!reference && !wire_bypass && !repairs) {
      require(!router.recoverCurrentPosition(task,32768,true,true,nullptr,false),
          "early incomplete-only repair broke a completed cycle participant");
      require(!router.route_stats.preempt_success && router.pending_route_todo.empty() &&
          fpga::isRouteComplete(y->wires.front()) && fpga::isRouteComplete(z->wires.front()) &&
          fpga::isRouteComplete(u->wires.front()), "rejected early repair mutated completed competitors");
      f.auditAll();
    }
    bool done=!reference && router.recoverCurrentPosition(task,32768,true);
    if(!reference && !done && task.to->outline.fixed)
      done=router.recoverCurrentPosition(task,fpga::UNBOUND_ROUTE_STATE_BUDGETS.back(),true);
    if(!repairs)first_probe=done;
    ++repairs;
    if(!done) {
      std::vector<pnr::RouteDesign::RouteTask> affected;std::string reason;bool wire_only=false;
      // The reference deliberately tests placement feasibility alone. The
      // production arm must run the complete early wire-recovery policy.
      require(router.moveUnfinishedCell(task,&affected,&task,&reason,false,reference?nullptr:&wire_only),"verified legal cycle move failed: "+reason);
      require(!wire_only,"early Moving policy cut completed competitors instead of using the free relocation");
      ++moves;
      for(auto& t:affected)require(router.routeNetTask(t),"cycle move left an incident input unfinished");
    }
    for(const auto& pending:router.pending_route_todo)require(std::any_of(tasks.begin(),tasks.end(),[&](const auto& t) {
      return pending.net==t.net && pending.from==t.from && pending.to==t.to;
    }),"cycle preempted an unrelated route");
    router.pending_route_todo.clear(); // Real victims are already in tasks, in production recovery order.
    f.auditAll();
  }
  std::cout<<"Moving four-way cycle reference="<<reference<<" bypass="<<wire_bypass<<" repairs="<<repairs
      <<" moves="<<moves<<" cuts="<<router.route_stats.preempt_success<<" complete="<<complete()<<'\n';
  require(complete(),"Moving repeats preempting in-place repairs instead of using its feasible destination move");
  if(!reference && wire_bypass)require(first_probe && moves==0 && !router.route_stats.preempt_success && x->tile.peer==&f.tile({45,11}),
      "free current-position recovery did not win over relocation/preemption");
  else require(repairs==1 && moves==1 && !router.route_stats.preempt_success &&
      x->tile.peer==&f.tile({53,18}),"cycle did not use one free relocation without displacing competitors");
  require(a->tile.peer==&f.tile({54,17}) && b->tile.peer==&f.tile({40,11}) && c->tile.peer==&f.tile({40,11}) &&
      y->tile.peer==&f.tile({44,12}) && z->tile.peer==&f.tile({44,16}) && u->tile.peer==&f.tile({50,10}),
      "cycle recovery moved a stationary endpoint");
  f.auditAll();
  }
}
