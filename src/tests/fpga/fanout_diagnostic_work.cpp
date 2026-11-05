#include "SourceInputFixture.h"
#include "debug.h"

#include <cstdio>
#include <cstdlib>
#include <optional>
#include <unistd.h>

using namespace routing_tests;
namespace {
struct Setting {
  const char* key;std::optional<std::string> old;
  Setting(const char* k,const char* value):key(k) {
    if(const char* p=std::getenv(k))old=p;
    if(value)setenv(k,value,1);else unsetenv(k);
  }
  ~Setting(){if(old)setenv(key,old->c_str(),1);else unsetenv(key);}
};
struct Capture {
  std::FILE* file=std::tmpfile();int saved=-1;
  Capture(){require(file,"cannot create diagnostic capture");std::fflush(stdout);saved=dup(STDOUT_FILENO);require(saved>=0 && dup2(fileno(file),STDOUT_FILENO)>=0,"cannot redirect diagnostics");}
  std::string finish(){std::fflush(stdout);dup2(saved,STDOUT_FILENO);close(saved);saved=-1;std::rewind(file);std::string s;char b[4096];while(size_t n=std::fread(b,1,sizeof(b),file))s.append(b,n);return s;}
  ~Capture(){if(saved>=0){std::fflush(stdout);dup2(saved,STDOUT_FILENO);close(saved);}std::fclose(file);}
};
void install(SourceInputFixture& f,rtl::Net& net,rtl::Inst* from,const char* out,
             rtl::Inst* to,const char* in,std::vector<fpga::Wire> route) {
  require(pnr::commitPreparedRoute(route),"cannot install diagnostic reference route");to->wires.push_back(std::move(route));
  auto id=fpga::attachNetRoute(net,*to,to->wires.size()-1,from,to,out,in,SourceInputFixture::port(*to,in).makeNetName());
  fpga::registerNetRouteTiles(net,to->wires.back(),id);
}
}

// Example 1 spent eight of thirty Moving samples formatting SRC owner tables:
// MOVE_DETAIL (coarse progress) accidentally enabled per-state fanout tracing.
// Use a real multi-hop fanout with a long registered background route crossing
// its candidate Tiles. Count owner-table builds printed by the expansion
// diagnostic, not seconds. Explicit net tracing must retain those tables;
// coarse detail must route the identical unique path without building any.
void fanout_diagnostic_work_regression(int mode) {
  SourceInputFixture f(false,0,8,20);
  auto& target=f.tile({19,2});
  for(int bit=0;bit<2;++bit)for(auto kind:{fpga::ELEMENT_LUT5,fpga::ELEMENT_LUT1}) {
    fpga::Element e;e.name="slot_pattern_"+std::to_string(bit)+"_"+std::to_string(kind);e.type=kind;e.bitmap_pos=bit;target.tile_type->elements.push_back(e);
  }
  target.tile_type->pin_map.nodes[{"LUT6","I0",7}].setBit(189);target.cb_type->local_input_nodes.setBit(189);
  auto* sink=f.cell("branch_receiver_71","LUT6",{{"I0",false}},{19,2},3);
  auto* background_driver=f.cell("background_driver_73","FDRE",{{"Q",true}},{1,0});
  auto* background_sink=f.cell("background_receiver_79","LUT6",{{"I0",false}},{19,2},7);
  auto* background=f.connect(*background_driver,"Q",*background_sink,"I0");
  auto& in=SourceInputFixture::port(*sink,"I0");in.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.source,"Q")));in.port_ref->designator=SourceInputFixture::port(*f.source,"Q").port_ref->designator;
  std::vector<fpga::Wire> trunk{f.pin({7,0},141,true),f.edge({7,0},{8,0},141,1000,1001,true),
      f.edge({8,0},{0,0},1001,1002,1003),f.terminal({0,0},1003,87),f.pin({0,0},87,false)};
  fpga::Coord prev{8,0};int input=1001;
  for(int x=9;x<20;++x){f.edge(prev,{x,0},input,1100+x,1200+x);prev={x,0};input=1200+x;}
  f.edge(prev,{19,1},input,1300,1301);f.edge({19,1},{19,2},1301,1302,1303);f.terminal({19,2},1303,87);
  std::vector<fpga::Coord> line{{1,0}};
  for(int x=8;x<20;++x)line.push_back({x,0});
  for(int x=19;x>=8;--x)line.push_back({x,1});
  for(int x=8;x<20;++x)line.push_back({x,2});
  std::vector<fpga::Wire> bg{f.pin({1,0},141,true)};
  for(size_t i=0;i+1<line.size();++i)bg.push_back(f.edge(line[i],line[i+1],i?2000+int(i)-1:141,2100+int(i),2000+int(i),i==0));
  bg.push_back(f.terminal({19,2},2000+int(line.size())-2,189));bg.push_back(f.pin({19,2},189,false));
  for(auto& cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  install(f,*f.signal,f.source,"Q",f.sink,"I0",std::move(trunk));
  install(f,*background,background_driver,"Q",background_sink,"I0",std::move(bg));f.auditAll();
  auto& r=f.tech.route;r.moving_stage=r.complete_routes_only=r.preserve_routes=true;r.route_preemption_enabled=r.route_deadends_enabled=false;
  r.iteration_limit=r.route_suffix_depth_limit=r.route_recursion_budget=200;r.indexSourceRoute(f.signal,f.source,"Q");
  pnr::RouteDesign::RouteTask task{f.source,sink,f.signal,"Q","I0",in.makeNetName()};task.fanout=true;
  Setting detail("SCALEPNR_ROUTE_MOVE_DETAIL",mode==1?"0":"1");
  Setting explicit_net("SCALEPNR_DEBUG_FANOUT_NET",mode==2?task.net_name.c_str():nullptr);
  Setting explicit_route("SCALEPNR_DEBUG_ROUTE_NET",nullptr),all_tasks("SCALEPNR_DEBUG_FANOUT_TASKS",nullptr);
  const int old_level=debug_level;debug_level=1;bool done=false;std::string text;
  {Capture capture;done=r.routeNetTask(task);text=capture.finish();}debug_level=old_level;
  require(done,"diagnostic mode changed fanout completion");f.auditAll();
  size_t tables=0,pos=0;while((pos=text.find("src_owners=",pos))!=std::string::npos){++tables;++pos;}
  std::printf("fanout_diagnostic_work mode=%d owner_table_builds=%zu edge_checks=%zu\n",mode,tables,r.route_stats.edge_trials);
  require(mode==2?tables>0:tables==0,"coarse Moving detail performed per-state SRC owner-table work");
  require(mode==1 || text.find("routeFanoutTask debug")!=std::string::npos,"explicit/coarse fanout diagnostics disappeared");
  const auto& b=f.signal->routes.back();const auto& route=b.owner->wires[b.route_index];
  require(fpga::isRouteComplete(route) && route.front().from==fpga::Coord{7,0} && route.back().from==fpga::Coord{19,2},"diagnostics changed route endpoints");
  std::vector<fpga::Coord> path{{7,0}},expected{{7,0}};
  for(const auto& w:route) {
    const bool hop=(w.type==fpga::Wire::WIRE_CROSSBAR && w.jump>=0) ||
        (w.type==fpga::Wire::WIRE_ROUTE_EDGE && w.from_node_type==fpga::CB_NODE_SRC && w.to_node_type==fpga::CB_NODE_DST);
    if(hop && w.from!=w.to) {require(path.back()==w.from,"diagnostics broke route continuity");path.push_back(w.to);}
  }
  for(int x=8;x<20;++x)expected.push_back({x,0});
  expected.push_back({19,1});expected.push_back({19,2});
  require(path==expected,"diagnostic settings changed the unique physical routing path");
}
