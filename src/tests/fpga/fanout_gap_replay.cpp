#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"
#include "route/RouteDesign.h"
#include "route/DockingCutChecks.h"
#include "route/RoutePassState.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unistd.h>

namespace {
using fpga::Coord;
using fpga::Tile;
using fpga::Wire;
using Route = std::vector<Wire>;

void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}
bool same(Coord a, Coord b) { return a.x == b.x && a.y == b.y; }

// Capture existing production diagnostics; restore caller settings on failure.
struct SourceRepairTrace {
  char path[64]="/tmp/scalepnr-source-repair-XXXXXX";
  std::map<std::string,std::string> old;
  explicit SourceRepairTrace(const std::string &net) {
    int fd=mkstemp(path);
    require(fd>=0,"cannot create source-repair trace"); close(fd);
    for (const char *key:{"SCALEPNR_AUDIT_SOURCE_REACHABILITY","SCALEPNR_SOURCE_REACHABILITY_LOG"})
      if (const char *value=std::getenv(key)) old[key]=value;
    setenv("SCALEPNR_AUDIT_SOURCE_REACHABILITY",net.c_str(),1);
    setenv("SCALEPNR_SOURCE_REACHABILITY_LOG",path,1);
  }
  ~SourceRepairTrace() {
    for (const char *key:{"SCALEPNR_AUDIT_SOURCE_REACHABILITY","SCALEPNR_SOURCE_REACHABILITY_LOG"}) {
      auto it=old.find(key);
      if(it==old.end()) unsetenv(key); else setenv(key,it->second.c_str(),1);
    }
    unlink(path);
  }
};

struct RepairDecisionTrace {
  std::FILE *file=std::tmpfile();
  int saved=dup(STDERR_FILENO);
  bool had_setting=std::getenv("SCALEPNR_DEBUG_ROUTE_REPAIR")!=nullptr;
  std::string old=had_setting?std::getenv("SCALEPNR_DEBUG_ROUTE_REPAIR"):"";
  RepairDecisionTrace() {
    require(file && saved>=0,"cannot capture repair decisions");
    std::fflush(stderr);
    require(dup2(fileno(file),STDERR_FILENO)>=0,"cannot redirect repair decisions");
    setenv("SCALEPNR_DEBUG_ROUTE_REPAIR","1",1);
  }
  ~RepairDecisionTrace() {
    std::fflush(stderr); dup2(saved,STDERR_FILENO); close(saved); std::fclose(file);
    if(had_setting) setenv("SCALEPNR_DEBUG_ROUTE_REPAIR",old.c_str(),1);
    else unsetenv("SCALEPNR_DEBUG_ROUTE_REPAIR");
  }
  std::string read() {
    std::fflush(stderr); std::rewind(file);
    std::string text; char buffer[4096];
    while(size_t n=std::fread(buffer,1,sizeof(buffer),file)) text.append(buffer,n);
    return text;
  }
};

// Numeric replay of the two-state failed repair recorded in Example 1.
// Preserve the victim's six paths, cut boundaries, outgoing choices and busy
// landings. All textual names are synthetic. Unrelated owner routes outside
// this frontier are short synthetic paths, not a copy of the complete chip.
// In particular, busy bits are NEVER installed without route owners.
struct Signal {
  Referable<rtl::Net> net;
  Referable<rtl::Cell> driver_cell;
  rtl::Inst driver;
  std::deque<Referable<rtl::Cell>> sink_cells;
  std::deque<rtl::Inst> sinks;
};

struct Fixture {
  fpga::Device &device = fpga::Device::current();
  std::map<int, std::unique_ptr<Signal>> signals;
  std::vector<Tile *> tiles;
  int auxiliary = 0;

  explicit Fixture(int height = 40, int width = 40) {
    device.tile_grid.clear();
    device.cb_types.clear();
    device.cb_types.reserve(256);
    device.grid_spec.size = {width, height};
    device.size_width = width; device.size_height = height;
    device.tile_grid.resize(width * height);
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
      auto &t = device.tile_grid[y * width + x];
      t.coord = t.cb_coord = {x, y};
    }
  }

  Tile &tile(Coord c) {
    auto *t = device.getTile(c.x, c.y);
    require(t != nullptr, "fixture coordinate outside grid");
    if (!t->cb_type) {
      require(device.cb_types.size() < device.cb_types.capacity(), "CB storage moved");
      const int id = device.cb_types.size();
      device.cb_types.emplace_back();
      auto &cb = device.cb_types.back();
      cb.type_id = cb.base_type_id = id;
      cb.name = "matrix_" + std::to_string(7001 + id * 31);
      t->cb_type = &cb;
      t->cb.type = &cb;
      t->full_name = "tile_" + std::to_string(c.y * device.size_width + c.x);
      tiles.push_back(t);
    }
    return *t;
  }

  void name(Coord c, fpga::CBNodeNameType kind, int id) {
    tile(c).cb_type->rememberNodeName(kind, id,
        "node_" + std::to_string((static_cast<unsigned>(id) * 2654435761U) ^
                                  (static_cast<unsigned>(kind) * 101U)));
  }

  using Node=std::tuple<int,int,int,int>;
  // Import only eligible numeric edges; resource names are generated locally.
  void arc(Node a,Node b) {
    auto [x,y,kind,id]=a; auto [nx,ny,nkind,nid]=b;
    auto &at=tile({x,y}); auto &to=tile({nx,ny}); auto &cb=*at.cb_type;
    name({x,y},static_cast<fpga::CBNodeNameType>(kind),id);
    name({nx,ny},static_cast<fpga::CBNodeNameType>(nkind),nid);
    if(kind==fpga::CB_NODE_SRC && nkind==fpga::CB_NODE_DST) {
      auto &jumps=cb.dst_by_src[id];
      for(const auto &j:jumps)
        if(same(j.delta,Coord{nx-x,ny-y}) && j.dsts.jump.testBit(nid)) return;
      fpga::CBType::ResolvedJump j{}; j.delta={nx-x,ny-y};
      j.target_tile_coord=true; j.target_cb_type_id=to.cb_type->type_id;
      j.dsts.jump.setBit(nid); jumps.push_back(j); return;
    }
    if(kind==fpga::CB_NODE_LOCAL && (x!=nx || y!=ny)) {
      fpga::CBType::ResolvedLocal l{}; l.delta={nx-x,ny-y};
      l.target_cb_type_id=to.cb_type->type_id;
      l.target_node_type=static_cast<fpga::CBNodeNameType>(nkind);
      l.target_nodes.setBit(nid); cb.local_by_local[id].push_back(l); return;
    }
    require(x==nx && y==ny,"unexpected cross-tile non-jump arc");
    if(kind==fpga::CB_NODE_LOCAL && nkind==fpga::CB_NODE_SRC) cb.local_src[id].jump.setBit(nid);
    else if(kind==fpga::CB_NODE_LOCAL && nkind==fpga::CB_NODE_LOCAL) cb.local_local[id].local.setBit(nid);
    else if(kind==fpga::CB_NODE_LOCAL && nkind==fpga::CB_NODE_JOINT) cb.local_joint[id].joint.setBit(nid);
    else if(kind==fpga::CB_NODE_DST && nkind==fpga::CB_NODE_SRC) cb.dst_src[id].jump.setBit(nid);
    else if(kind==fpga::CB_NODE_DST && nkind==fpga::CB_NODE_LOCAL) cb.dst_local[id].local.setBit(nid);
    else if(kind==fpga::CB_NODE_DST && nkind==fpga::CB_NODE_JOINT) cb.dst_joint[id].joint.setBit(nid);
    else if(kind==fpga::CB_NODE_SRC && nkind==fpga::CB_NODE_JOINT) cb.src_joint[id].joint.setBit(nid);
    else if(kind==fpga::CB_NODE_JOINT && nkind==fpga::CB_NODE_SRC) cb.joint_src[id].jump.setBit(nid);
    else if(kind==fpga::CB_NODE_JOINT && nkind==fpga::CB_NODE_LOCAL) cb.joint_local[id].local.setBit(nid);
    else if(kind==fpga::CB_NODE_JOINT && nkind==fpga::CB_NODE_JOINT) cb.joint_joint[id].joint.setBit(nid);
    else require(false,"unknown snapshot arc");
  }

  void loadFrontier(const std::string &file,size_t expected,bool grouped=true) {
    std::ifstream edges(std::filesystem::path(__FILE__).parent_path()/file);
    require(bool(edges),"missing frontier snapshot");
    int x,y,kind,id,nx,ny,nkind,nid; size_t count=0;
    while(edges>>x>>y>>kind>>id) {
      size_t targets=1;
      if(grouped) require(bool(edges>>targets) && targets<4096,"invalid frontier group");
      for(size_t i=0;i<targets;++i) {
        require(bool(edges>>nx>>ny>>nkind>>nid),"truncated frontier edge");
        arc({x,y,kind,id},{nx,ny,nkind,nid}); ++count;
      }
    }
    require(edges.eof() && count==expected,"incomplete frontier snapshot");
  }

  Wire pin(Coord c, int local, bool output) {
    name(c, fpga::CB_NODE_LOCAL, local);
    Wire w;
    w.type = Wire::WIRE_TILE_PIN;
    w.from = w.to = w.resource = c;
    w.local = local;
    w.pin_dir = output ? 1 : 0;
    return w;
  }

  Wire edge(Coord from, Coord to, int local, int src, int dst, int pos = 1) {
    auto &cb = *tile(from).cb_type;
    auto &target = tile(to);
    name(from, pos == 0 ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST, local);
    name(from, fpga::CB_NODE_SRC, src);
    name(to, fpga::CB_NODE_DST, dst);
    if (pos == 0) cb.local_src[local].jump.setBit(src);
    else cb.dst_src[local].jump.setBit(src);
    auto &jumps = cb.dst_by_src[src];
    if (jumps.empty()) {
      fpga::CBType::ResolvedJump jump{};
      jump.delta = to - from;
      // Fixture coordinates are physical grid coordinates. Resolve them
      // directly, including Tiles introduced by the later real-path import.
      jump.target_tile_coord = true;
      jump.target_cb_type_id = target.cb_type->type_id;
      jump.dsts.jump.setBit(dst);
      jumps.push_back(jump);
    } else {
      require(jumps.size() == 1 && same(jumps.front().delta, to - from) &&
              jumps.front().dsts.jump.testBit(dst), "inconsistent synthetic physical jump");
    }
    target.incoming_dst_nodes.setBit(dst);
    Wire w;
    w.from = from; w.to = to; w.local = local;
    w.jump = w.route_jump = src; w.dst = dst; w.pos = pos;
    w.owns_dst = pos != 2;
    return w;
  }

  void terminal(Route &r, Coord c, int dst, int local, int joint = -1, int joint2 = -1) {
    auto &cb = *tile(c).cb_type;
    if (joint < 0) cb.dst_local[dst].local.setBit(local);
    else {
      cb.dst_joint[dst].joint.setBit(joint2 < 0 ? joint : joint2);
      if (joint2 >= 0) {
        name(c, fpga::CB_NODE_JOINT, joint2);
        cb.joint_joint[joint2].joint.setBit(joint);
      }
      name(c, fpga::CB_NODE_JOINT, joint);
      cb.joint_local[joint].local.setBit(local);
    }
    Wire w;
    w.from = w.to = c; w.local = dst; w.pos = 1;
    w.joint = joint; w.joint2 = joint2;
    r.push_back(w);
    r.push_back(pin(c, local, false));
  }

  Signal &signal(int id) {
    auto &s = signals[id];
    if (!s) {
      s = std::make_unique<Signal>();
      s->net.name = "signal_" + std::to_string(id);
      s->driver_cell.type = "logic_731";
      s->driver_cell.name = "cell_" + std::to_string(id) + "_source";
      s->driver.cell_ref.set(&s->driver_cell);
    }
    return *s;
  }

  Route &install(int id, Route r) {
    auto &s = signal(id);
    if (s.sinks.empty()) s.driver.tile.set(&device.tile_grid[r.front().from.y*device.size_width+r.front().from.x]);
    else require(same(s.sinks.front().wires.front().front().from,r.front().from) &&
                 s.sinks.front().wires.front().front().local==r.front().local,
                 "one signal has inconsistent physical source endpoints");
    s.sinks.emplace_back();
    auto &sink = s.sinks.back();
    s.sink_cells.emplace_back();
    s.sink_cells.back().type = "logic_731";
    s.sink_cells.back().name = "cell_" + std::to_string(id) + "_sink_" +
                              std::to_string(s.sinks.size());
    sink.cell_ref.set(&s.sink_cells.back());
    sink.tile.set(&device.tile_grid[r.back().from.y*device.size_width+r.back().from.x]);
    // The test starts with committed routes. Reconstruct every lease from
    // those fragments, including boundaries on both sides of explicit gaps.
    for (const auto &w : r) {
      auto &t = tile(w.from);
      if (w.type == Wire::WIRE_TILE_PIN) {
        t.cb.local.local.setBit(w.local);
        t.pin_state.leased_nodes.setBit(w.local);
      } else if (w.type == Wire::WIRE_CROSSBAR) {
        if (w.pos != 0 && w.owns_dst) t.cb.dst.jump.setBit(w.local);
        if (w.jump >= 0) t.cb.src.jump.setBit(w.jump);
        if (w.joint >= 0) t.cb.joint.jump.setBit(w.joint);
        if (w.joint2 >= 0) t.cb.joint.jump.setBit(w.joint2);
        if (w.owns_landing) tile(w.to).cb.dst.jump.setBit(w.dst);
      }
    }
    sink.wires.push_back(std::move(r));
    const auto index = fpga::attachNetRoute(s.net, sink, 0, &s.driver, &sink,
        "out", "in", s.net.name + "_branch_" + std::to_string(s.sinks.size() - 1));
    fpga::registerNetRouteTiles(s.net, sink.wires.front(), index);
    return sink.wires.front();
  }

  Route prefix(size_t binding, size_t count) {
    auto &old = signal(162539).sinks[binding].wires.front();
    Route r(old.begin(), old.begin() + count);
    for (auto &w : r) w.shared = true;
    return r;
  }

  void victim() {
    Route r{pin({25,28},149,true), edge({25,28},{22,29},149,246,3843,0),
            edge({22,29},{22,29},3843,23,23)};
    terminal(r,{22,29},23,86,12,26); install(162539,std::move(r));
    r=prefix(0,2);
    r.push_back(edge({22,29},{21,30},3843,3619,3619,2));
    r.push_back(edge({21,30},{25,34},3619,819,819));
    r.push_back(edge({25,34},{22,34},819,3846,3842));
    r.push_back(edge({22,34},{22,33},3842,249,244));
    terminal(r,{22,33},244,83); install(162539,std::move(r));
    r=prefix(1,3);
    r.push_back(edge({21,30},{16,30},3619,3846,3842,2));
    r.push_back(edge({16,30},{16,29},3842,249,244));
    r.push_back(edge({16,29},{16,28},244,248,242));
    r.push_back(edge({16,28},{15,27},242,3809,3809));
    r.push_back(edge({15,27},{15,25},3809,247,241));
    r.push_back(edge({15,25},{21,25},241,1024,1024));
    r.push_back(edge({21,25},{16,25},1024,3841,3846));
    r.push_back(edge({16,25},{15,24},3846,3809,3809));
    r.push_back(edge({15,24},{11,20},3809,3537,3537));
    r.push_back(edge({11,20},{11,14},3537,161,161));
    r.push_back(edge({11,14},{12,14},161,256,256));
    r.push_back(edge({12,14},{12,15},256,16,19));
    r.push_back(edge({12,15},{12,17},19,64,64));
    terminal(r,{12,17},64,106,8,22); install(162539,std::move(r));
    r=prefix(1,4);
    r.push_back(edge({25,34},{25,28},819,163,163,2));
    r.push_back(edge({25,28},{22,27},163,3840,3845));
    r.push_back(edge({22,27},{21,25},3845,3808,3808));
    r.push_back(edge({21,25},{15,21},3808,3536,3536));
    r.push_back(edge({15,21},{11,17},3536,3536,3536));
    r.push_back(edge({11,17},{12,18},3536,240,259));
    r.push_back(edge({12,18},{15,17},259,262,260));
    terminal(r,{15,17},260,80,36,51); install(162539,std::move(r));
    r=prefix(1,5);
    r.push_back(edge({22,34},{21,33},3842,3840,3845,2));
    r.push_back(edge({21,33},{16,32},3845,3808,3808));
    r.push_back(edge({16,32},{12,28},3808,3536,3536));
    r.push_back(edge({12,28},{15,27},3536,736,736));
    r.push_back(edge({15,27},{12,27},736,3841,3846));
    r.push_back(edge({12,27},{11,28},3846,3616,3616));
    r.push_back(edge({11,28},{11,27},3616,247,241));
    r.push_back(edge({11,27},{15,27},241,1024,1024));
    r.push_back(edge({15,27},{15,29},1024,64,64));
    terminal(r,{15,29},64,121); install(162539,std::move(r));
    r=prefix(1,6);
    r.push_back(edge({22,33},{21,33},244,3843,3848,2));
    r.push_back(edge({21,33},{16,32},3848,3840,3845));
    r.push_back(edge({16,32},{15,32},3845,3841,3846));
    r.push_back(edge({15,32},{12,32},3846,3842,3847));
    r.push_back(edge({12,32},{11,32},3847,3843,3848));
    r.push_back(edge({11,32},{11,33},3848,22,26));
    r.push_back(edge({11,33},{12,32},26,262,260));
    terminal(r,{12,32},260,97); install(162539,std::move(r));
  }

  // A short owner route supplies a captured foreign transit or takeoff.
  // Partial owners have a genuine retained prefix, GAP and downstream island.
  void owner(int id, Coord from, int local, int src, Coord to, int dst,
             bool partial, bool takeoff = false, int copies = 1, bool extend = false) {
    const int tag = ++auxiliary;
    auto &s = signal(id);
    const Coord origin = s.sinks.empty() ? Coord{2,tag}
        : s.sinks.front().wires.front().front().from;
    Route base;
    if (takeoff) base.push_back(pin(from,local,true));
    else {
      base = {pin(origin,3000,true), edge(origin,{3,tag},3000,3000+tag,3001,0)};
      if (partial) {
        base.back().owns_landing = true;
        Wire gap; gap.type=Wire::WIRE_GAP;
        gap.from={3,tag}; gap.to=from;
        gap.from_node_type=gap.to_node_type=fpga::CB_NODE_DST;
        gap.from_node=3001; gap.to_node=local;
        base.push_back(gap);
      } else base.push_back(edge({3,tag},from,3001,3002,local));
    }
    base.push_back(edge(from,to,local,src,dst,takeoff ? 0 : 1));
    if (extend) {
      base.push_back(edge(to,{5,tag},dst,3300,3301));
      to={5,tag}; dst=3301;
    }
    for (int i=0;i<copies;++i) {
      Route r=base;
      terminal(r,to,dst,3100+tag*4+i);
      install(id,std::move(r));
    }
  }

  void blockers() {
    owner(163828,{16,28},150,193,{16,25},193,false,true);
    owner(162752,{16,28},1025,242,{16,27},246,false);
    owner(162401,{16,28},3846,247,{16,27},241,false);
    owner(163546,{16,28},261,256,{21,28},256,false);
    owner(162269,{16,28},977,737,{21,27},737,false);
    owner(162269,{16,28},246,3842,{15,28},3847,true);
    // SRC3809 at (16,28) is claimed by the actual second requester below.
    owner(161803,{22,28},24,17,{22,29},20,true);
    owner(162484,{22,28},150,65,{22,30},65,false,true);
    owner(163213,{22,28},545,97,{22,34},97,false);
    owner(163981,{22,28},817,161,{22,21},161,true);
    owner(161776,{22,28},1029,193,{22,25},193,false);
    owner(164370,{22,28},977,242,{22,27},246,true);
    owner(161893,{22,28},3809,256,{25,28},256,false);
    owner(166407,{22,28},261,258,{25,28},262,false);
    owner(166556,{22,28},154,545,{25,29},545,false,true);
    owner(163213,{22,28},545,737,{25,27},737,false);
    owner(166412,{22,28},65,817,{26,32},817,false);
    owner(161652,{22,28},193,977,{26,23},977,true);
    owner(166408,{22,28},161,1027,{32,28},1029,false);
    owner(162184,{26,28},1025,17,{26,29},20,true,false,4);
    owner(162366,{16,32},3377,3617,{15,33},3617,true);
    owner(162184,{21,28},3847,3617,{16,29},3617,true);
    edge({16,28},{22,28},242,1026,1025);
    edge({22,28},{26,28},1025,1026,1025);
    edge({22,28},{16,32},1025,3377,3377);
    edge({22,28},{21,28},1025,3842,3847);
    // Busy choices still need the exact incoming DST's topology arcs.
    for (int src : {193,242,247,256,737,3809,3842})
      tile({16,28}).cb_type->dst_src[242].jump.setBit(src);
    for (int src : {17,65,97,161,193,242,256,258,545,737,817,977,1027})
      tile({22,28}).cb_type->dst_src[1025].jump.setBit(src);
  }

  // Exact additional arcs observed by the full-device, input-preserving
  // source audit. No occupancy is removed, no new physical jumps invented.
  // The two LOCALs are routing intermediates, not newly created cell pins.
  void real_bypasses() {
    // Reuse the existing source tree through (25,34)/DST819, then reconnect
    // the first retained gap end. The first three physical hops already exist.
    name({25,34},fpga::CB_NODE_LOCAL,165);
    tile({25,34}).cb_type->dst_local[819].local.setBit(165);
    edge({25,34},{15,34},165,3078,3079,0);
    edge({15,34},{4,34},3079,3078,3079);
    edge({4,34},{4,35},3079,3617,545);
    edge({4,35},{11,31},545,977,977);
    edge({11,31},{15,27},977,977,977);
    edge({15,27},{15,25},977,247,241);

    // Separate complete source-to-sink alternative: ten physical hops, using
    // a free source exit and the already-reserved sink LOCAL106.
    edge({25,28},{36,28},149,1025,1028,0);
    edge({36,28},{36,25},1028,192,192);
    edge({36,25},{25,25},192,3072,3077);
    edge({25,25},{15,25},3077,3072,3077);
    name({15,25},fpga::CB_NODE_LOCAL,178);
    tile({15,25}).cb_type->dst_local[3077].local.setBit(178);
    edge({15,25},{11,30},178,3376,3376,0);
    edge({11,30},{4,30},3376,3074,3072);
    edge({4,30},{4,23},3072,161,161);
    edge({4,23},{11,19},161,977,977);
    edge({11,19},{12,18},977,737,737);
    edge({12,18},{12,17},737,242,246);
    tile({12,17}).cb_type->dst_local[246].local.setBit(106);
    rebuild();
  }

  void rebuild() { for (auto &cb : device.cb_types) cb.rebuildOutgoingSrcs(); }

  void cut(const std::vector<fpga::RouteCutNode> &cuts, int requester, Route replacement,
           size_t expected_victims) {
    rebuild();
    const auto victims=fpga::findNetRouteCutVictims(cuts);
    require(victims.size()==expected_victims,"cut affected unexpected route dependencies");
    const auto released=fpga::previewNetRouteCuts(victims,cuts,true);
    if (released.empty()) {
      std::ostringstream details;
      for (const auto &v : victims) fpga::dumpNetRouteHistory(*v.net,details);
      throw std::runtime_error("cut had no validated release footprint for requester " +
          std::to_string(requester) + "\n" + details.str());
    }
    require(pnr::commitPreparedRouteWithPreemption(replacement,released,[&] {
      return fpga::unrouteNetRoutesFromNodes(victims,cuts,true);
    }),"complete requester replacement failed atomic preemption");
    require(fpga::isRouteComplete(install(requester,std::move(replacement))),
            "requester was left unfinished");
  }

  void preempt() {
    // First cut: release the last two hops plus old Grounding, retaining LOCAL106.
    Route r{pin({12,13},3200,true),edge({12,13},{12,14},3200,3200,3201,0),
            edge({12,14},{12,15},3201,16,19)};
    terminal(r,{12,15},19,3202);
    cut({{{12,14},fpga::CB_NODE_SRC,16},{{12,15},fpga::CB_NODE_DST,19}},
        94436,std::move(r),1);
    require(signal(162539).sinks[2].wires.front().size()==16,"first cut differs from replay");
    // A second victim of the same four-resource cut, already partial, owns SRC20.
    owner(93635,{15,27},1027,20,{15,28},24,true,false,1,true);
    const Wire origin=signal(162184).sinks.front().wires.front().front();
    r={origin,edge(origin.from,{17,29},origin.local,3400,3401,0),
       edge({17,29},{16,28},3401,3200,3616),
       edge({16,28},{15,27},3616,3809,3809),edge({15,27},{15,28},3809,20,24)};
    terminal(r,{15,28},24,3202);
    cut({{{16,28},fpga::CB_NODE_SRC,3809},{{15,27},fpga::CB_NODE_DST,3809},
         {{15,27},fpga::CB_NODE_SRC,20},{{15,28},fpga::CB_NODE_DST,24}},
        162184,std::move(r),2);
    require(signal(162539).sinks[2].wires.front().size()==15,"second cut differs from replay");
    // The final snapshot also contains a later, independently preempted sibling.
    r={pin({12,26},3200,true),edge({12,26},{12,27},3200,3200,3201,0),
       edge({12,27},{11,28},3201,3616,3616)};
    terminal(r,{11,28},3616,3202);
    cut({{{12,27},fpga::CB_NODE_SRC,3616}},920476,std::move(r),1);
    r={pin({15,26},3200,true),edge({15,26},{15,27},3200,3200,3201,0),
       edge({15,27},{15,29},3201,64,64)};
    terminal(r,{15,29},64,3202);
    cut({{{15,27},fpga::CB_NODE_SRC,64}},920506,std::move(r),1);
  }

  std::string state(size_t expected_driver_conflicts=0) {
    std::ostringstream out;
    std::vector<rtl::Net *> nets;
    for (const auto &[id,s] : signals) {
      nets.push_back(&s->net);
      fpga::dumpNetRouteHistory(s->net,out);
    }
    size_t driver_conflicts=0;
    for (auto *t : tiles) {
      const auto a=fpga::auditTileCongestion(*t,out,nets);
      driver_conflicts+=a.conflicting_driver_nodes;
      if (a.orphan_leases || a.missing_leases || a.missing_registrations ||
          a.stale_registrations)
        throw std::runtime_error("fixture contains orphan leases or broken route registrations\n"+out.str());
    }
    if(driver_conflicts!=expected_driver_conflicts)
      throw std::runtime_error("fixture contains unexpected conflicting drivers\n"+out.str());
    return out.str();
  }

  std::string unaffected_routes() {
    std::ostringstream out;
    for (const auto &[id,s] : signals) {
      if (id!=162539) { fpga::dumpNetRouteHistory(s->net,out); continue; }
      for (size_t i=0;i<s->sinks.size();++i) if (i!=2) {
        out << "sibling " << i << '\n';
        for (const auto &w:s->sinks[i].wires.front())
          out << w.type << ' ' << w.from.x << ' ' << w.from.y << ' ' << w.to.x << ' '
              << w.to.y << ' ' << w.local << ' ' << w.jump << ' ' << w.dst << ' '
              << w.pos << ' ' << w.joint << ' ' << w.joint2 << ' ' << w.shared << ' '
              << w.owns_dst << ' ' << w.owns_landing << ' ' << w.from_node_type << ' '
              << w.from_node << ' ' << w.to_node_type << ' ' << w.to_node << '\n';
      }
    }
    return out.str();
  }
};

void check_gap(const Wire &w, Coord from, int src, Coord to, int dst, int type) {
  require(w.type==Wire::WIRE_GAP && same(w.from,from) && same(w.to,to) &&
          w.from_node_type==fpga::CB_NODE_DST && w.from_node==src &&
          w.to_node_type==type && w.to_node==dst,"retained gap differs from captured failure");
}

// Independently check every programmed arc, not just isRouteComplete's state
// marker. Explicit LOCAL continuations must be connected to both neighbors.
void check_complete_path(Fixture &f, const Route &route, int expected_hops=10) {
  using Node=std::tuple<int,int,int,int>;
  auto node=[](Coord at,int kind,int id) { return Node{at.x,at.y,kind,id}; };
  Node current=node(route.front().from,fpga::CB_NODE_LOCAL,route.front().local);
  int hops=0;
  auto edge=[&](Node from,Node to) {
    require(current==from,"repaired route has a discontinuity");
    const auto [x,y,kind,id]=from;
    const auto [nx,ny,next_kind,next_id]=to;
    auto &tile=f.tile({x,y}); auto &cb=*tile.cb_type;
    bool connected=false;
    if (kind==fpga::CB_NODE_SRC && next_kind==fpga::CB_NODE_DST) {
      for (const auto &target:f.device.resolveJumpTargets(tile,id))
        connected|=target.tile && same(target.tile->coord,{nx,ny}) && target.dst_node==next_id;
      hops+=(x!=nx || y!=ny);
    } else {
      require(x==nx && y==ny,"non-jump arc crossed a Tile boundary");
      if (kind==fpga::CB_NODE_LOCAL && next_kind==fpga::CB_NODE_SRC)
        connected=cb.local_src[id].jump.testBit(next_id);
      if (kind==fpga::CB_NODE_DST && next_kind==fpga::CB_NODE_SRC)
        connected=cb.dst_src[id].jump.testBit(next_id);
      if (kind==fpga::CB_NODE_DST && next_kind==fpga::CB_NODE_LOCAL)
        connected=cb.dst_local[id].local.testBit(next_id);
      if (kind==fpga::CB_NODE_LOCAL && next_kind==fpga::CB_NODE_LOCAL)
        connected=cb.local_local[id].local.testBit(next_id);
      if (kind==fpga::CB_NODE_DST && next_kind==fpga::CB_NODE_JOINT)
        connected=cb.dst_joint[id].joint.testBit(next_id);
      if (kind==fpga::CB_NODE_JOINT && next_kind==fpga::CB_NODE_JOINT)
        connected=cb.joint_joint[id].joint.testBit(next_id);
      if (kind==fpga::CB_NODE_JOINT && next_kind==fpga::CB_NODE_LOCAL)
        connected=cb.joint_local[id].local.testBit(next_id);
    }
    require(connected,"repaired route programmed an absent topology arc");
    current=to;
  };
  for (size_t i=1;i+1<route.size();++i) {
    const auto &w=route[i];
    if (w.type==Wire::WIRE_ROUTE_EDGE) {
      edge(node(w.from,w.from_node_type,w.from_node),node(w.to,w.to_node_type,w.to_node));
    } else {
      require(w.type==Wire::WIRE_CROSSBAR,"unexpected fragment on the imported path");
      auto input=node(w.from,w.pos?fpga::CB_NODE_DST:fpga::CB_NODE_LOCAL,w.local);
      for(int joint:{w.joint2,w.joint}) if(joint>=0) {
        auto next=node(w.from,fpga::CB_NODE_JOINT,joint);
        edge(input,next); input=next;
      }
      if (w.jump>=0) {
        auto output=node(w.from,fpga::CB_NODE_SRC,w.jump);
        edge(input,output); edge(output,node(w.to,fpga::CB_NODE_DST,w.dst));
      } else {
        edge(input,node(route.back().from,fpga::CB_NODE_LOCAL,route.back().local));
      }
    }
  }
  require(current==node(route.back().from,fpga::CB_NODE_LOCAL,route.back().local) && hops==expected_hops,
          "repair did not reach its sink through the verified path length");
}

// Independent enumeration of the tiny free frontier, with explicit counts.
// All rejected busy nodes must have foreign, live owners; none is a hint bit.
void check_frontier(Fixture &f) {
  using Node=std::tuple<int,int,int>;
  std::vector<Node> queue{{16,28,242}};
  std::set<Node> seen(queue.begin(),queue.end());
  int busy_src=0, free_src=0, busy_dst=0, free_dst=0;
  for(size_t i=0;i<queue.size();++i) {
    const auto [x,y,dst]=queue[i]; auto &t=f.tile({x,y});
    const auto *sources=t.cb_type->srcNodes(fpga::CB_NODE_DST,dst);
    require(sources && sources->size()==(i==0 ? 8 : 16),"frontier candidate count differs from replay");
    for(int src:*sources) {
      auto foreign=[&](Tile &at,fpga::CBNodeNameType type,int node) {
        const auto owners=fpga::findNetOwnersByNode(at,type,node);
        require(!owners.empty(),"frontier busy bit has no owner");
        for(const auto &o:owners) require(o.net!=&f.signal(162539).net,
            "frontier accidentally blocked by the victim's own tree");
      };
      if(t.cb.src.jump.testBit(src)) { ++busy_src; foreign(t,fpga::CB_NODE_SRC,src); continue; }
      ++free_src;
      const auto targets=f.device.resolveJumpTargets(t,src);
      require(targets.size()==1,"fixture changed the number of physical jump targets");
      const auto &target=targets.front();
      if(target.tile->cb.dst.jump.testBit(target.dst_node)) {
        ++busy_dst; foreign(*target.tile,fpga::CB_NODE_DST,target.dst_node);
        for(const auto &o:fpga::findNetOwnersByNode(*target.tile,fpga::CB_NODE_DST,target.dst_node)) {
          const auto &b=o.net->routes[o.binding_index];
          require(fpga::hasRouteGaps(b.owner->wires[b.route_index]),
                  "blocked landing is not retained by a partial route");
        }
      } else {
        ++free_dst;
        Node n{target.tile->coord.x,target.tile->coord.y,target.dst_node};
        if(seen.insert(n).second) queue.push_back(n);
      }
    }
  }
  require(queue==std::vector<Node>{{16,28,242},{22,28,1025}} &&
          busy_src==20 && free_src==4 && busy_dst==3 && free_dst==1,
          "free reachability differs from the recorded two-state dead end");
}

struct Reachability {
  bool gap_end = false;
  bool sink_tile = false;
  size_t states = 0;
};

// An optimistic, read-only upper bound for a wider repair. Start at the real
// driver and allow ALL resources owned by that source to be reused, even if
// that would require replacing retained segments. Ignore joint occupancy and
// accept any arrival at the sink Tile without requiring Grounding. Thus a
// negative result proves there is no free bypass in this synthetic topology;
// a positive result alone does NOT prove a legal replacement route exists.
Reachability source_reachability(Fixture &f) {
  const auto &binding = f.signal(162539).net.routes[2];
  using Node = std::tuple<int, int, int, int>;
  std::vector<Node> queue{{25,28,fpga::CB_NODE_LOCAL,149}};
  std::set<Node> seen(queue.begin(), queue.end());
  auto available = [&](Tile &tile, fpga::CBNodeNameType type, int node) {
    const bool busy = type == fpga::CB_NODE_SRC
        ? tile.cb.src.jump.testBit(node) : tile.cb.dst.jump.testBit(node);
    if (!busy) return true;
    const auto owners = fpga::findNetOwnersByNode(tile, type, node);
    require(!owners.empty(), "source reachability found an unowned busy node");
    return std::all_of(owners.begin(), owners.end(), [&](const auto &ref) {
      const auto &other = ref.net->routes[ref.binding_index];
      return other.from == binding.from && other.from_port == binding.from_port;
    });
  };
  Reachability result;
  for (size_t i = 0; i < queue.size(); ++i) {
    const auto [x,y,kind,node] = queue[i];
    result.gap_end |= x == 15 && y == 25 && kind == fpga::CB_NODE_DST && node == 241;
    result.sink_tile |= x == 12 && y == 17 && kind == fpga::CB_NODE_DST;
    auto *tile = f.device.getTile(x,y);
    if (!tile || !tile->cb_type) continue;
    const auto *sources = tile->cb_type->srcNodes(
        static_cast<fpga::CBNodeNameType>(kind), node);
    if (!sources) continue;
    for (int src : *sources) {
      if (!available(*tile, fpga::CB_NODE_SRC, src)) continue;
      for (const auto &target : f.device.resolveJumpTargets(*tile, src)) {
        if (!target.tile || target.dst_node < 0 ||
            !available(*target.tile, fpga::CB_NODE_DST, target.dst_node)) continue;
        Node next{target.tile->coord.x, target.tile->coord.y,
                  fpga::CB_NODE_DST, target.dst_node};
        if (seen.insert(next).second) queue.push_back(next);
      }
    }
  }
  result.states = queue.size();
  return result;
}
} // namespace

// Second failure: the exact gap is trapped, but a reached source-tree path
// ends at a free SRC whose landing is foreign transit, directly feeding the
// requester's reserved sink. Coordinates, numeric arcs and cut boundaries
// below come from that snapshot; all textual identifiers remain synthetic.
static void run_gap_grounding(bool free_alternative, bool joint_terminal) {
  Fixture f;
  Route r{f.pin({22,32},156,true),f.edge({22,32},{22,33},156,19,22,0),
      f.edge({22,33},{16,33},22,3079,3075),
      f.edge({16,33},{15,33},3075,3846,3842),
      f.edge({15,33},{15,31},3842,195,195),
      f.edge({15,31},{15,30},195,244,248),
      f.edge({15,30},{15,29},248,249,244),
      f.edge({15,29},{15,27},244,194,194),
      f.edge({15,27},{11,22},194,3538,3538),
      f.edge({11,22},{11,21},3538,248,242),
      f.edge({11,21},{12,21},242,256,256),
      f.edge({12,21},{16,21},256,1024,1024),
      f.edge({16,21},{16,19},1024,192,192),
      f.edge({16,19},{16,17},192,192,192),
      f.edge({16,17},{16,17},192,245,240),
      f.edge({16,17},{16,15},240,195,195),
      f.edge({16,15},{12,11},195,3539,3539),
      f.edge({12,11},{11,12},3539,3618,3618),
      f.edge({11,12},{11,13},3618,18,21),
      f.edge({11,13},{11,14},21,18,21),
      f.edge({11,14},{11,16},21,66,66)};
  f.terminal(r,{11,16},66,85);
  auto &route=f.install(162316,std::move(r));
  auto &s=f.signal(162316);
  f.rebuild(); f.state();
  for (const auto &cuts:std::vector<std::vector<fpga::RouteCutNode>>{
      {{{12,11},fpga::CB_NODE_SRC,3618},{{11,14},fpga::CB_NODE_SRC,66}},
      {{{11,21},fpga::CB_NODE_SRC,256}},{{{15,29},fpga::CB_NODE_SRC,194}}}) {
    const auto victims=fpga::findNetRouteCutVictims(cuts);
    require(victims.size()==1 && fpga::unrouteNetRoutesFromNodes(victims,cuts,true),
            "could not reproduce retained transit holes");
  }
  require(std::count_if(route.begin(),route.end(),[](const Wire &w) {
    return w.type==Wire::WIRE_GAP;
  })==3,"snapshot must retain three disconnected gaps");
  std::vector<const Wire *> gaps;
  for(const auto &w:route) if(w.type==Wire::WIRE_GAP) gaps.push_back(&w);
  check_gap(*gaps[0],{15,29},244,{11,22},3538,fpga::CB_NODE_DST);
  check_gap(*gaps[1],{11,21},242,{16,21},1024,fpga::CB_NODE_DST);
  check_gap(*gaps[2],{12,11},3539,{11,16},85,fpga::CB_NODE_LOCAL);
  // Eight recorded exits of the first gap. Each busy node has an actual owner.
  f.owner(161940,{15,27},194,900,{17,27},900,true);
  f.owner(162972,{15,29},901,243,{17,29},901,true);
  f.owner(164144,{15,29},902,248,{17,30},902,true);
  f.owner(163027,{15,29},903,259,{17,31},903,false,true);
  f.owner(164043,{16,28},738,904,{18,28},904,true);
  f.owner(165022,{15,29},905,1028,{18,29},905,true);
  f.owner(162708,{15,29},906,3810,{18,30},906,true);
  f.owner(166407,{15,29},907,3843,{18,31},907,true);
  for(int src:{243,248,259,1028,3810,3843})
    f.tile({15,29}).cb_type->dst_src[244].jump.setBit(src);
  f.edge({15,29},{16,28},244,738,738);
  // Exact numeric source path reached in production, including LOCAL bridges.
  f.edge({16,33},{12,33},3075,3079,3075);
  f.edge({12,33},{12,33},3075,23,23);
  f.tile({12,33}).cb_type->dst_local[23].local.setBit(165);
  f.tile({12,33}).cb_type->local_local[165].local.setBit(184);
  f.edge({12,33},{12,27},184,162,162,0);
  f.edge({12,27},{4,27},162,3078,1030);
  f.edge({4,27},{4,20},1030,162,162);
  f.edge({4,20},{4,14},162,162,162);
  f.edge({4,14},{11,14},162,1028,1026);
  f.edge({11,14},{11,16},1026,66,66);
  // Recorded contested transit. Two dependent bindings check atomic cuts and
  // rescheduling; the real snapshot had one physical owner of this fragment.
  f.owner(166424,{11,16},66,98,{11,22},98,true,false,2,true);
  f.rebuild();
  const auto before=f.state();
  std::ostringstream audit;
  require(!fpga::auditNetRouteReachability(s.net,0,audit) &&
          audit.str().find("limited=1")==std::string::npos,
          "a free bypass unexpectedly exists in the captured terminal case");
  require(!fpga::repairNetRouteGaps(s.net,0) && f.state()==before,
          "free-only repair must fail without changing victims");
  pnr::RouteDesign router; router.fpga=&f.device; router.fanout_stage=true;
  pnr::RouteDesign::RouteTask task{.from=&s.driver,.to=&s.sinks[0],.net=&s.net,
      .from_port="out",.to_port="in",.net_name=s.net.routes[0].route_name,
      .fanout=true,.endpoints_prepared=true};
  router.route_preemption_enabled=false;
  require(!router.routeNetTask(task) && f.state()==before && router.pending_route_todo.empty(),
          "disabled preemption changed the trapped source tree");
  router.route_preemption_enabled=true;
  auto &transit=f.signal(166424).net;
  transit.route_protected=true;
  require(!router.routeNetTask(task) && f.state()==before && router.pending_route_todo.empty(),
          "repair cut a protected terminal transit owner");
  transit.route_protected=false;
  f.tile({11,16}).cb_type->dst_local[66].local.clearBit(85);
  require(!router.routeNetTask(task) && f.state()==before && router.pending_route_todo.empty(),
          "repair preempted before proving a complete connection to the sink");
  f.tile({11,16}).cb_type->dst_local[66].local.setBit(85);
  if (joint_terminal) {
    auto &cb=*f.tile({11,16}).cb_type;
    cb.dst_local[66].local.clearBit(85);
    cb.dst_joint[66].joint.setBit(5);
    cb.joint_joint[5].joint.setBit(19);
    cb.joint_local[19].local.setBit(85);
    Route blocker{f.pin({18,18},140,true),f.edge({18,18},{11,16},140,68,68,0)};
    f.terminal(blocker,{11,16},68,87,19);
    f.install(166425,std::move(blocker)); f.rebuild();
    const auto blocked=f.state();
    require(!router.routeNetTask(task) && f.state()==blocked && router.pending_route_todo.empty(),
            "eligible DST was cut before rejecting a protected second JOINT");
    // Keep the protected route intact; offer a distinct free second JOINT.
    cb.joint_joint[5].joint.clearBit(19);
    cb.joint_joint[5].joint.setBit(20);
    cb.joint_local[20].local.setBit(85);
    // Packed input reservations must also prevent the provisional cut.
    auto &tile=f.tile({11,16});
    tile.input_joint_reservations_initialized=true;
    NodeMask reserved_joints; reserved_joints.setBit(20);
    tile.input_joint_reservations.emplace_back(nullptr,reserved_joints);
    const auto reserved=f.state();
    require(!router.routeNetTask(task) && f.state()==reserved && router.pending_route_todo.empty(),
            "repair ignored a packed JOINT reservation");
    tile.input_joint_reservations.clear();
  }
  if (free_alternative) {
    f.edge({11,14},{11,16},1026,67,67);
    f.tile({11,16}).cb_type->dst_local[67].local.setBit(85);
    f.rebuild();
  }
  std::ostringstream transit_before;
  fpga::dumpNetRouteHistory(transit,transit_before);
  require(router.routeNetTask(task) && fpga::isRouteComplete(route),
          "reached SRC66 cannot Ground through eligible transit DST66");
  check_complete_path(f,route,9);
  require(router.pending_route_todo.size()==(free_alternative?0:2),
          "repair did not reschedule every preempted transit binding");
  for(const auto &task:router.pending_route_todo)
    require(task.net==&f.signal(166424).net,"repair preempted an unrelated frontier owner");
  require(route[route.size()-2].local==(free_alternative?67:66) && route.back().local==85,
          "repair did not use the recorded direct Grounding path");
  if (free_alternative) {
    std::ostringstream after; fpga::dumpNetRouteHistory(transit,after);
    require(after.str()==transit_before.str(),"free alternative did not suppress preemption");
  } else {
    for(const auto &sink:f.signal(166424).sinks)
      require(fpga::hasRouteGaps(sink.wires.front()),"victim was fully discarded rather than partially cut");
  }
  f.state();
  std::printf("gap grounding replay: complete=1 free_alternative=%d two_joints=%d requeued=%zu ownership=clean\n",
      free_alternative,joint_terminal,router.pending_route_todo.size());
}

void fanout_gap_grounding_regression() {
  run_gap_grounding(false,false);
  run_gap_grounding(true,false);
  run_gap_grounding(false,true);
}

// Captured binding-24 failure: retain both holes and every fragment between
// them. Import the seven-hop reached prefix and its contested final jump.
// Only unrelated owner endpoints are shortened; names are synthetic hashes.
enum class FinalJumpGuard { None, Takeoff, Grounding, Joint, OutsideGrounding };
static void run_final_jump_replay(bool free_alternative=false,
                                 FinalJumpGuard guard=FinalJumpGuard::None,
                                 bool shared_victim=false) {
  Fixture f;
  auto gap=[](Coord a,int dst,Coord b,int kind,int node) {
    Wire w; w.type=Wire::WIRE_GAP; w.from=a; w.to=b;
    w.from_node_type=fpga::CB_NODE_DST; w.from_node=dst;
    w.to_node_type=kind; w.to_node=node; return w;
  };
  Route r{f.pin({12,25},154,true),f.edge({12,25},{15,27},154,545,545,0),
      f.edge({15,27},{15,20},545,161,161),f.edge({15,20},{15,14},161,161,161),
      f.edge({15,14},{4,14},161,3073,3078),f.edge({4,14},{5,14},3078,257,261),
      f.edge({5,14},{11,15},261,545,545),f.edge({11,15},{11,21},545,97,97),
      f.edge({11,21},{12,21},97,258,262),f.edge({12,21},{15,21},262,260,263)};
  r.back().owns_landing=true;
  r.push_back(gap({15,21},263,{11,25},fpga::CB_NODE_DST,98));
  r.push_back(f.edge({11,25},{15,30},98,818,818));
  r.push_back(f.edge({15,30},{15,23},818,162,162));
  r.push_back(f.edge({15,23},{21,28},162,818,818));
  r.push_back(f.edge({21,28},{16,28},818,3845,3841));
  r.push_back(f.edge({16,28},{16,25},3841,194,194));
  r.push_back(f.edge({16,25},{12,25},194,3076,3073));
  r.push_back(f.edge({12,25},{12,32},3073,97,97));
  r.push_back(f.edge({12,32},{15,33},97,545,545));
  r.push_back(f.edge({15,33},{15,32},545,242,246));
  r.back().owns_landing=true;
  r.push_back(gap({15,32},246,{15,29},fpga::CB_NODE_LOCAL,96));
  r.push_back(f.pin({15,29},96,false));
  require(r.size()==22,"final-jump replay lost captured fragments");
  Route source_sibling(r.begin(),r.begin()+10);
  for(auto &w:source_sibling) { w.shared=true; w.owns_landing=false; }
  f.terminal(source_sibling,{15,21},263,3106);
  auto &route=f.install(162399,std::move(r));
  f.install(162399,std::move(source_sibling));
  auto &s=f.signal(162399);
  f.edge({12,25},{16,30},154,817,817,0);
  f.edge({16,30},{21,29},817,737,737);
  f.edge({21,29},{16,28},737,3809,3809);
  f.edge({16,28},{12,32},3809,3376,3376);
  f.edge({12,32},{4,32},3376,3073,1029);
  f.edge({4,32},{15,32},1029,1027,1029);
  f.edge({15,32},{15,30},1029,193,193);
  // A two-jump terminal approach INSIDE the Grounding window is legal.
  // Protect arbitrary intermediate transit by putting this detour outside it.
  const Coord landing=guard==FinalJumpGuard::OutsideGrounding
      ?Coord{15,29-fpga::GROUNDING_RADIUS-1}:Coord{15,29};
  f.edge({15,30},landing,193,247,241);
  if (guard==FinalJumpGuard::OutsideGrounding)
    f.edge(landing,{15,29},241,3002,241);
  auto &sink=f.tile({15,29});
  sink.cb_type->dst_local[241].local.setBit(96);
  // Real contested neighborhood: SRC247 and DST241 belong to one foreign
  // transit binding. Its remote source/sink are deliberately reduced.
  Route owner;
  if (guard==FinalJumpGuard::Takeoff) {
    owner={f.pin({15,30},3100,true),f.edge({15,30},landing,3100,247,241,0)};
  } else {
    owner={f.pin({13,30},3100,true),f.edge({13,30},{15,30},3100,3000,3537,0),
           f.edge({15,30},landing,3537,247,241)};
  }
  if (guard==FinalJumpGuard::Grounding) f.terminal(owner,landing,241,3101);
  else {
    owner.push_back(f.edge(landing,{12,28},241,3808,3808));
    f.terminal(owner,{12,28},3808,3101);
  }
  auto &victim=f.install(165018,std::move(owner));
  if (shared_victim) {
    Route sibling(victim.begin(),victim.begin()+3);
    for(auto &w:sibling) w.shared=true;
    sibling.push_back(f.edge({15,29},{16,29},241,3003,3003,2));
    f.terminal(sibling,{16,29},3003,3102);
    f.install(165018,std::move(sibling));
    Route unaffected(victim.begin(),victim.begin()+2);
    for(auto &w:unaffected) w.shared=true;
    unaffected.push_back(f.edge({15,30},{17,31},3537,3004,3004,2));
    f.terminal(unaffected,{17,31},3004,3103);
    f.install(165018,std::move(unaffected));
  }
  if (guard==FinalJumpGuard::Joint) {
    sink.cb_type->dst_local[241].local.clearBit(96);
    sink.cb_type->dst_joint[241].joint.setBit(56);
    sink.cb_type->joint_joint[56].joint.setBit(41);
    sink.cb_type->joint_local[41].local.setBit(96);
    Route protected_tail{f.pin({18,28},3104,true),
        f.edge({18,28},{15,29},3104,3006,247,0)};
    f.terminal(protected_tail,{15,29},247,125,41,56);
    f.install(163231,std::move(protected_tail));
  }
  f.rebuild();
  const auto before=f.state();
  SourceRepairTrace trace(s.net.name);
  require(!fpga::repairNetRouteGaps(s.net,0) && f.state()==before,
          "free-only replay must fail without changing either route");
  std::ifstream trace_file(trace.path);
  const std::string log((std::istreambuf_iterator<char>(trace_file)),{});
  require(log.find("tile=15,30 kind=4 node=193 cost=42 hops=7 expanded=1")!=std::string::npos,
          "replay did not reach the captured seven-hop final-jump predecessor");
  pnr::RouteDesign router; router.fpga=&f.device; router.fanout_stage=true;
  pnr::RouteDesign::RouteTask task{.from=&s.driver,.to=&s.sinks[0],.net=&s.net,
      .from_port="out",.to_port="in",.net_name=s.net.routes[0].route_name,
      .fanout=true,.endpoints_prepared=true};
  router.route_preemption_enabled=false;
  require(!router.routeNetTask(task) && f.state()==before && router.pending_route_todo.empty(),
          "disabled preemption changed the final-jump fixture");
  router.route_preemption_enabled=true;
  if (guard!=FinalJumpGuard::None) {
    require(!router.routeNetTask(task) && f.state()==before && router.pending_route_todo.empty(),
            "final Grounding cut an endpoint or arbitrary intermediate transit");
    return;
  }
  int previews=0;
  auto reject_preview=[&](const std::vector<fpga::RouteCutNode> &,bool,
                          std::vector<fpga::RouteCutNode> &) { ++previews; return false; };
  require(!fpga::repairNetRouteGaps(s.net,0,[]{return true;},32768,200,{},reject_preview) &&
          previews==0 && f.state()==before,"cancelled final jump changed routing");
  int commits=0;
  auto allow_preview=[&](const std::vector<fpga::RouteCutNode> &cuts,bool commit,
                         std::vector<fpga::RouteCutNode> &released) {
    if(commit) {++commits;return false;}
    released=fpga::previewNetRouteCuts(fpga::findNetRouteCutVictims(cuts),cuts,true);
    return !released.empty();
  };
  // Eligibility may be queried before a complete path is known. Permit the
  // previews so this really tests the hop bound, not a rejecting callback.
  require(!fpga::repairNetRouteGaps(s.net,0,{},32768,7,{},allow_preview) &&
          commits==0 && f.state()==before,"final jump bypassed the total hop bound");
  require(!fpga::repairNetRouteGaps(s.net,0,{},32768,200,{},reject_preview) &&
          f.state()==before,"rejected final-jump preview changed routing");
  require(previews>0,"BUG: reached DST193 never previews occupied final SRC247");
  auto &transit=f.signal(165018).net;
  transit.route_protected=true;
  require(!router.routeNetTask(task) && f.state()==before && router.pending_route_todo.empty(),
          "final jump cut a protected owner");
  transit.route_protected=false;
  sink.cb_type->dst_local[241].local.clearBit(96);
  require(!router.routeNetTask(task) && f.state()==before && router.pending_route_todo.empty(),
          "final jump was cut without a complete terminal path");
  sink.cb_type->dst_local[241].local.setBit(96);
  if (free_alternative) {
    f.edge({15,30},{15,29},193,3005,3005);
    sink.cb_type->dst_local[3005].local.setBit(96); f.rebuild();
  }
  std::ostringstream old_victim; fpga::dumpNetRouteHistory(transit,old_victim);
  require(router.routeNetTask(task) && fpga::isRouteComplete(route),
          "BUG: reached DST193 never tries occupied transit SRC247 leading to sink DST241");
  check_complete_path(f,route,8);
  check_complete_path(f,s.sinks[1].wires.front(),9);
  require(router.pending_route_todo.size()==(free_alternative?0:shared_victim?2:1),
          "final-jump cut requeued the wrong set of dependent bindings");
  require(route[route.size()-2].local==(free_alternative?3005:241),
          "repair selected the wrong terminal path");
  for(const auto &task:router.pending_route_todo)
    require(task.net==&transit,"final jump preempted an unrelated net");
  if (free_alternative) {
    std::ostringstream after; fpga::dumpNetRouteHistory(transit,after);
    require(after.str()==old_victim.str(),"free final jump did not suppress preemption");
  } else {
    require(fpga::hasRouteGaps(victim),"final jump discarded the entire victim");
    if(shared_victim) {
      require(fpga::hasRouteGaps(f.signal(165018).sinks[1].wires.front()),
              "shared final-jump dependent was not cut");
      check_complete_path(f,f.signal(165018).sinks[2].wires.front(),2);
    }
  }
  f.state();
  std::printf("final jump: eight-hop completion, free=%d shared=%d requeued=%zu ownership=clean\n",
      free_alternative,shared_victim,router.pending_route_todo.size());
}

void fanout_final_jump_regression() {
  run_final_jump_replay(false,FinalJumpGuard::Takeoff);
  run_final_jump_replay(false,FinalJumpGuard::Grounding);
  run_final_jump_replay(false,FinalJumpGuard::Joint);
  run_final_jump_replay(false,FinalJumpGuard::OutsideGrounding);
  run_final_jump_replay();
  run_final_jump_replay(true);
  run_final_jump_replay(false,FinalJumpGuard::None,true);
}

// Example 1, fourth chunk, binding 28: five retained gaps; free source search
// reaches (45,35)/DST99 after 15 hops. Completing through SRC262 -> (49,34)
// DST260/SRC544 -> (50,35)/DST544 -> LOCAL96 requires TWO terminal jumps.
// The live diagnostic validated this complete 17-hop path and all four cuts,
// but repair only considered preemption on the final jump and rejected it.
// Preserve the 32 requester fragments and the exact witnessed bypass. The
// foreign owner's remote history is shortened to a GAP; its contested island
// and terminal tail retain the recorded arcs. Eleven unrelated bindings are
// omitted, not replaced with fake busy bits. Every name is abstract.
void fanout_two_jump_grounding_regression() {
  Fixture f(80,80);
  auto gap=[&](Route &r,Coord a,int dst,Coord b,int kind,int node) {
    r.back().owns_landing=true;
    Wire w; w.type=Wire::WIRE_GAP; w.from=a; w.to=b;
    w.from_node_type=fpga::CB_NODE_DST; w.from_node=dst;
    w.to_node_type=kind; w.to_node=node; r.push_back(w);
  };
  Route r{f.pin({57,34},154,true),f.edge({57,34},{58,34},154,256,256,0),
      f.edge({58,34},{62,33},256,736,736,2)};
  gap(r,{62,33},736,{62,39},fpga::CB_NODE_DST,19);
  for(int y=39;y<41;++y) r.push_back(f.edge({62,y},{62,y+1},19,16,19));
  gap(r,{62,41},19,{62,43},fpga::CB_NODE_DST,19);
  for(int y=43;y<51;++y) r.push_back(f.edge({62,y},{62,y+1},19,16,19));
  r.push_back(f.edge({62,51},{62,54},19,64,64));
  r.push_back(f.edge({62,54},{62,55},64,16,19));
  r.push_back(f.edge({62,55},{62,56},19,16,19));
  gap(r,{62,56},19,{57,59},fpga::CB_NODE_DST,3072);
  r.push_back(f.edge({57,59},{57,57},3072,193,193));
  r.push_back(f.edge({57,57},{54,56},193,3809,3809));
  r.push_back(f.edge({54,56},{54,55},3809,247,241));
  r.push_back(f.edge({54,55},{54,54},241,241,245));
  r.push_back(f.edge({54,54},{54,53},245,241,245));
  r.push_back(f.edge({54,53},{57,51},245,736,736));
  gap(r,{57,51},736,{57,43},fpga::CB_NODE_DST,3808);
  r.push_back(f.edge({57,43},{57,37},3808,160,160));
  r.push_back(f.edge({57,37},{53,33},160,3536,3536));
  r.push_back(f.edge({53,33},{50,33},3536,3841,3846));
  r.push_back(f.edge({50,33},{45,33},3846,3074,3072));
  gap(r,{45,33},3072,{50,35},fpga::CB_NODE_LOCAL,96);
  r.push_back(f.pin({50,35},96,false));
  require(r.size()==32,"two-jump replay lost requester fragments");
  Route sibling(r.begin(),r.begin()+2);
  for(auto &w:sibling) w.shared=true;
  f.terminal(sibling,{58,34},256,3900);
  auto &route=f.install(163960,std::move(r));
  f.install(163960,std::move(sibling));
  auto &s=f.signal(163960);
  const auto sibling_before=s.sinks[1].wires.front();

  f.edge({62,33},{58,32},736,3808,3808);
  f.edge({58,32},{54,28},3808,3536,3536);
  auto local=[&](Coord c,int dst,int id) {
    f.name(c,fpga::CB_NODE_LOCAL,id);
    f.tile(c).cb_type->dst_local[dst].local.setBit(id);
  };
  local({54,28},3536,183);
  f.edge({54,28},{54,21},183,163,163,0);
  f.edge({54,21},{54,15},163,163,163);
  f.edge({54,15},{45,15},163,3075,3080);
  f.edge({45,15},{50,11},3080,979,979);
  local({50,11},979,128);
  f.edge({50,11},{45,15},128,3379,3379,0);
  f.edge({45,15},{45,21},3379,99,99);
  f.edge({45,21},{45,28},99,99,99);
  f.edge({45,28},{45,34},99,99,99);
  local({45,34},99,128);
  f.edge({45,34},{45,28},128,163,163,0);
  f.edge({45,28},{45,29},163,22,26);
  local({45,29},26,128);
  f.edge({45,29},{45,35},128,99,99,0);
  f.edge({45,35},{49,34},99,262,260);
  f.edge({49,34},{50,35},260,544,544);
  auto &sink=f.tile({50,35});
  sink.cb_type->dst_local[544].local.setBit(96);

  Route owner{f.pin({63,43},145,true),f.edge({63,43},{62,44},145,3616,3616,0)};
  gap(owner,{62,44},3616,{49,21},fpga::CB_NODE_DST,979);
  owner.push_back(f.edge({49,21},{50,22},979,547,547));
  owner.push_back(f.edge({50,22},{50,24},547,67,67));
  owner.push_back(f.edge({50,24},{50,31},67,99,99));
  owner.push_back(f.edge({50,31},{45,35},99,3379,3379));
  owner.push_back(f.edge({45,35},{49,34},3379,262,260));
  owner.push_back(f.edge({49,34},{50,35},260,544,544));
  owner.push_back(f.edge({50,35},{54,39},544,816,816));
  gap(owner,{54,39},816,{50,41},fpga::CB_NODE_DST,3072);
  owner.push_back(f.edge({50,41},{50,40},3072,247,241));
  f.terminal(owner,{50,40},241,120);
  auto &victim=f.install(166452,std::move(owner));
  f.rebuild();
  const auto before=f.state(); // Audit leases against actual route owners.
  require(!fpga::repairNetRouteGaps(s.net,0) && f.state()==before,
          "free-only repair changed the captured two-jump obstruction");
  pnr::RouteDesign router; router.fpga=&f.device; router.fanout_stage=true;
  pnr::RouteDesign::RouteTask task{.from=&s.driver,.to=&s.sinks[0],.net=&s.net,
      .from_port="out",.to_port="in",.net_name=s.net.routes[0].route_name,
      .fanout=true,.endpoints_prepared=true};
  router.route_preemption_enabled=false;
  require(!router.routeNetTask(task) && f.state()==before,
          "disabled preemption changed a two-jump Grounding");
  router.route_preemption_enabled=true;
  f.signal(166452).net.route_protected=true;
  require(!router.routeNetTask(task) && f.state()==before,
          "two-jump Grounding cut a protected owner");
  f.signal(166452).net.route_protected=false;
  sink.cb_type->dst_local[544].local.clearBit(96);
  require(!router.routeNetTask(task) && f.state()==before && router.pending_route_todo.empty(),
          "two-jump Grounding cut transit before proving terminal continuity");
  sink.cb_type->dst_local[544].local.setBit(96);
  int previews=0;
  auto reject_preview=[&](const std::vector<fpga::RouteCutNode> &,bool,
                          std::vector<fpga::RouteCutNode> &) { ++previews; return false; };
  require(!fpga::repairNetRouteGaps(s.net,0,[]{return true;},32768,200,{},reject_preview) &&
          previews==0 && f.state()==before,"cancelled two-jump repair changed routing");
  int commits=0;
  auto allow_preview=[&](const std::vector<fpga::RouteCutNode> &cuts,bool commit,
                         std::vector<fpga::RouteCutNode> &released) {
    if(commit) {++commits;return false;}
    released=fpga::previewNetRouteCuts(fpga::findNetRouteCutVictims(cuts),cuts,true);
    return !released.empty();
  };
  require(!fpga::repairNetRouteGaps(s.net,0,{},32768,16,{},allow_preview) &&
          commits==0 && f.state()==before,"two-jump repair bypassed the 17-hop path limit");
  require(router.routeNetTask(task) && fpga::isRouteComplete(route),
          "BUG: validated two-jump Grounding is never attempted during gap repair");
  check_complete_path(f,route,17);
  check_complete_path(f,s.sinks[1].wires.front(),1);
  require(s.sinks[1].wires.front().size()==sibling_before.size(),
          "two-jump repair changed its completed source sibling");
  for(size_t i=0;i<sibling_before.size();++i) {
    const auto &a=sibling_before[i], &b=s.sinks[1].wires.front()[i];
    require(a.type==b.type && same(a.from,b.from) && same(a.to,b.to) &&
            a.local==b.local && a.jump==b.jump && a.dst==b.dst && a.pos==b.pos &&
            a.joint==b.joint && a.joint2==b.joint2,
            "two-jump repair moved a completed sibling arc");
  }
  require(router.pending_route_todo.size()==1 && router.pending_route_todo.front().net==&f.signal(166452).net,
          "two-jump Grounding did not requeue exactly the affected owner");
  require(fpga::hasRouteGaps(victim) && victim.front().local==145 && victim.back().local==120,
          "two-jump Grounding discarded the victim endpoints");
  f.state();
  std::printf("two-jump Grounding: 17-hop replacement, five old gaps removed, one transit owner requeued, ownership clean\n");
}

// A single preemption cuts two branches to DIFFERENT right boundaries. The
// parent's longer cut removes the only owner of a DST still borrowed by the
// sibling's retained island. This is the first corrupting operation in the
// recorded run-4 failure, not the later repair that detects foreign ownership.
void fanout_cut_island_regression(bool reverse) {
  Fixture f(64,64);
  Route trunk{f.pin({54,31},153,true),f.edge({54,31},{54,31},153,245,240,0),
              f.edge({54,31},{57,31},240,261,258),
              f.edge({57,31},{57,33},258,66,66)};
  Route parent=trunk;
  f.terminal(trunk,{57,33},66,130);
  f.install(781,trunk);
  for(auto &w:parent) w.shared=true;
  // Normalize the omitted upstream siblings' ownership to this trunk.
  parent.push_back(f.edge({57,33},{57,34},66,18,21,2));
  parent.push_back(f.edge({57,34},{53,34},21,3077,3074));
  parent.push_back(f.edge({53,34},{53,35},3074,22,26));
  Route sibling=parent;
  for(auto &w:sibling) w.shared=true;
  parent.push_back(f.edge({53,35},{53,35},26,23,23));
  parent.push_back(f.edge({53,35},{54,35},23,257,261));
  f.terminal(parent,{54,35},261,99);
  sibling.push_back(f.edge({53,35},{50,35},26,3846,3842,2));
  sibling.push_back(f.edge({50,35},{45,35},3842,3077,3074));
  sibling.push_back(f.edge({45,35},{44,35},3074,3845,3841));
  sibling.push_back(f.edge({44,35},{40,35},3841,3076,3073));
  sibling.push_back(f.edge({40,35},{37,34},3073,3810,3810));
  f.terminal(sibling,{37,34},3810,105,48,63);
  f.install(781,reverse?sibling:parent);
  f.install(781,reverse?parent:sibling);
  auto &net=f.signal(781).net;
  const size_t si=reverse?1:2, pi=reverse?2:1;
  auto &s=f.signal(781).sinks[si].wires.front();
  auto &p=f.signal(781).sinks[pi].wires.front();
  f.rebuild(); f.state();
  require(p.size()==11 && s.size()==14,"cut-island geometry changed");
  const std::vector<fpga::RouteCutNode> cuts{
      {{57,33},fpga::CB_NODE_SRC,18},{{53,35},fpga::CB_NODE_SRC,257}};
  const auto victims=fpga::findNetRouteCutVictims(cuts);
  require(victims.size()==2,"unexpected cut-island victims");
  const auto preview=fpga::previewNetRouteCuts(victims,cuts,true);
  require(std::none_of(preview.begin(),preview.end(),[](const auto &node) {
    return same(node.tile,{53,35}) && node.type==fpga::CB_NODE_DST && node.node==26;
  }),"preview incorrectly promises the borrowed landing");
  require(fpga::unrouteNetRoutesFromNodes(victims,cuts,true),"recorded cut rejected");
  require(p.size()==6 && s.size()==13,"recorded unequal cut extents changed");
  check_gap(p[4],{57,33},66,{54,35},99,fpga::CB_NODE_LOCAL);
  check_gap(s[4],{57,33},66,{53,34},3074,fpga::CB_NODE_DST);
  require(f.tile({53,35}).cb.dst.jump.testBit(26),
          "BUG: unequal partial cuts released retained fork's borrowed DST");
  require(!fpga::findNetOwnersByNode(f.tile({53,35}),fpga::CB_NODE_DST,26).empty(),
          "retained fork's borrowed DST has no owner after cut");
  require(f.tile({53,34}).cb.src.jump.testBit(22),"retained island lost its SRC");
  require(!f.tile({57,33}).cb.src.jump.testBit(18) &&
          !f.tile({53,35}).cb.src.jump.testBit(257),"contested SRCs not freed");
  f.state();
  // Extend the hole exactly as in the later real failure. The new right
  // boundary must still have a real lease, not resurrect an already free DST.
  const std::vector<fpga::RouteCutNode> more{{{53,34},fpga::CB_NODE_DST,3074}};
  require(fpga::unrouteNetRoutesFromNodes(fpga::findNetRouteCutVictims(more),more,true),
          "second cut rejected");
  check_gap(s[4],{57,33},66,{53,35},26,fpga::CB_NODE_DST);
  f.state();
  // A small synthetic bypass proves repair/teardown operate on real ownership.
  f.edge({57,33},{53,35},66,3001,26);
  f.rebuild();
  require(fpga::repairNetRouteGaps(net,si) && fpga::isRouteComplete(s),
          "retained branch cannot repair through valid boundary");
  f.state();
  require(fpga::unrouteNetRoute(net,pi),"parent teardown failed");
  require(fpga::isRouteComplete(s),"parent teardown damaged repaired sibling");
  f.state();
  require(fpga::unrouteNetRoute(net,si),"sibling teardown failed");
  require(!f.tile({53,35}).cb.dst.jump.testBit(26),"last borrower leaked DST");
  f.state();
  std::puts("cut island: unequal cuts preserve borrowed landing, repair and teardown");
}

// Captured fourth-chunk failure: a same-CB connection loses its two-hop
// middle to a new Takeoff. Free search cannot leave its retained prefix.
// The live read-only probe proved a THREE-hop replacement starting with an
// eligible occupied SRC and ending through FOUR JOINTs (the last two belong
// to the requester's detached tail). Preserve the seven requester fragments,
// the actual 15-fragment transit owner and the witnessed bypass. The owner's
// 29 unrelated bindings and unrelated blocked alternatives are omitted;
// normalize its shared-prefix ownership because those replicas are omitted.
// All textual node/cell/net names are abstract; every busy bit has a route.
// Example 1: repairing one branch must not release a sibling's borrowed DST
// in a shared island AFTER a gap. Import the recorded 13/30-fragment routes
// and actual replacement path; omit unrelated congestion and sibling routes.
// Every node name comes from Fixture::name, never from a device vendor.
void fanout_island_landing_regression() {
  Fixture f(80,80);
  Route trunk{f.pin({57,31},159,true),f.edge({57,31},{58,31},159,257,261,0)};
  f.terminal(trunk,{58,31},261,82,49,64);
  f.install(751,trunk);
  Route parent(trunk.begin(),trunk.begin()+2);
  for(auto &w:parent) w.shared=true;
  parent.push_back(f.edge({58,31},{58,30},261,242,246,2));
  parent.push_back(f.edge({58,30},{57,29},246,3809,3809));
  parent.push_back(f.edge({57,29},{54,30},3809,3616,3616));
  parent.push_back(f.edge({54,30},{53,31},3616,246,3843));
  parent.push_back(f.edge({53,31},{50,32},3843,3619,3619));
  Route sibling=parent;
  for(auto &w:sibling) w.shared=true;
  parent.push_back(f.edge({50,32},{45,32},3619,3079,3075));
  parent.push_back(f.edge({45,32},{45,34},3075,67,67));
  parent.push_back(f.edge({45,34},{44,35},67,3619,3619));
  parent.push_back(f.edge({44,35},{45,36},3619,547,547));
  f.terminal(parent,{45,36},547,94);
  auto &p=f.install(751,parent);
  sibling.push_back(f.edge({50,32},{54,36},3619,819,819,2));
  sibling.push_back(f.edge({54,36},{50,40},819,3379,3379));
  sibling.push_back(f.edge({50,40},{45,40},3379,3079,3075));
  sibling.push_back(f.edge({45,40},{41,40},3075,3079,3075));
  sibling.push_back(f.edge({41,40},{44,39},3075,262,260));
  sibling.push_back(f.edge({44,39},{49,39},260,1024,1024));
  sibling.push_back(f.edge({49,39},{44,43},1024,3376,3376));
  sibling.push_back(f.edge({44,43},{41,44},3376,3616,3616));
  sibling.push_back(f.edge({41,44},{40,43},3616,3809,3809));
  sibling.push_back(f.edge({40,43},{40,42},3809,247,241));
  sibling.push_back(f.edge({40,42},{40,40},241,192,192));
  sibling.push_back(f.edge({40,40},{41,39},192,736,736));
  sibling.push_back(f.edge({41,39},{37,35},736,3536,3536));
  sibling.push_back(f.edge({37,35},{37,29},3536,160,160));
  sibling.push_back(f.edge({37,29},{37,29},160,245,240));
  sibling.push_back(f.edge({37,29},{37,28},240,244,248));
  sibling.push_back(f.edge({37,28},{36,27},248,3840,3845));
  sibling.push_back(f.edge({36,27},{36,27},3845,245,240));
  sibling.push_back(f.edge({36,27},{40,27},240,1030,1027));
  sibling.push_back(f.edge({40,27},{40,33},1027,99,99));
  sibling.push_back(f.edge({40,33},{41,32},99,262,260));
  f.terminal(sibling,{41,32},260,113);
  auto &s=f.install(751,sibling);
  f.rebuild();
  require(p.size()==13 && s.size()==30,"island replay changed recorded route geometry");
  auto cut=[&](std::vector<fpga::RouteCutNode> nodes) {
    require(fpga::unrouteNetRoutesFromNodes(fpga::findNetRouteCutVictims(nodes),nodes,true),
            "could not make recorded partial cut");
  };
  cut({{{57,29},fpga::CB_NODE_SRC,3616},{{54,30},fpga::CB_NODE_DST,3616}});
  cut({{{45,32},fpga::CB_NODE_DST,3075},{{45,34},fpga::CB_NODE_DST,67}});
  require(p.size()==10 && s.size()==29,"island replay changed recorded cut geometry");
  check_gap(p[4],{57,29},3809,{53,31},3843,fpga::CB_NODE_DST);
  check_gap(p[6],{50,32},3619,{44,35},3619,fpga::CB_NODE_DST);
  // Cuts now give each retained tail independent ownership. Restore the
  // recorded, still-valid shared landing arrangement here to continue testing
  // SOURCE REPLACEMENT independently of that stronger cut invariant. The
  // parent owns this DST throughout; audit before replacement proves it.
  require(p[5].owns_landing && s[5].owns_landing && s[6].owns_dst,
          "partial cuts did not independently retain their islands");
  s[5].owns_landing=false;
  s[6].owns_dst=false;
  f.state();
  require(p[5].owns_landing && !s[5].owns_landing && !s[6].owns_dst,
          "island replay did not reproduce borrowed branching DST");
  Route obstruction{f.pin({57,29},151,true),f.edge({57,29},{54,30},151,3616,3616,0)};
  f.terminal(obstruction,{54,30},3616,130);
  f.install(752,std::move(obstruction));
  // Actual free source replacement from the captured repair, including its
  // DST -> LOCAL -> SRC bridge. These arcs are topology only, not leases.
  f.edge({58,30},{58,28},246,193,193);
  f.edge({58,28},{62,27},193,737,737);
  f.edge({62,27},{58,25},737,3809,3809);
  f.edge({58,25},{58,19},3809,161,161);
  f.edge({58,19},{50,19},161,3073,3078);
  f.edge({50,19},{50,18},3078,247,241);
  f.edge({50,18},{50,17},241,241,245);
  f.name({50,17},fpga::CB_NODE_LOCAL,183);
  f.tile({50,17}).cb_type->dst_local[245].local.setBit(183);
  f.edge({50,17},{50,23},183,99,99,0);
  f.edge({50,23},{45,28},99,3379,3379);
  f.edge({45,28},{45,34},3379,99,99);
  f.edge({45,34},{44,35},99,3619,3619);
  f.rebuild(); f.state();
  auto &net=f.signal(751).net;
  const auto before=f.state();
  require(!fpga::repairNetRouteGaps(net,1,[]{return true;}) && f.state()==before,
          "cancelled island replacement changed leases");
  require(fpga::repairNetRouteGaps(net,1) && fpga::isRouteComplete(p),
          "recorded source replacement did not finish");
  check_complete_path(f,p,14);
  require(f.tile({50,32}).cb.dst.jump.testBit(3619),
          "BUG: source replacement released a surviving fork's DST beyond a gap");
  require(!fpga::findNetOwnersByNode(f.tile({50,32}),fpga::CB_NODE_DST,3619).empty(),
          "surviving island landing has no owner");
  require(s[5].owns_landing && fpga::hasRouteGaps(s),
          "repair did not transfer the detached island landing to its sibling");
  f.state();
  // Reproduce the next preemption extending the gap. It must release the old
  // landing only when ALL its users are selected, never leave a foreign overlap.
  cut({{{53,31},fpga::CB_NODE_SRC,3619},{{50,32},fpga::CB_NODE_DST,3619}});
  require(!f.tile({50,32}).cb.dst.jump.testBit(3619),
          "cut did not release the contested island landing");
  f.state();
  require(fpga::unrouteNetRoute(net,2),"could not remove retained sibling");
  require(fpga::isRouteComplete(p),"sibling teardown damaged repaired parent");
  check_complete_path(f,p,14);
  f.state();
  std::puts("island landing: source replacement, second cut and teardown preserve ownership");
}

// Run-5 ownership audit: binding 75 replaces its source path while binding 54
// still uses (62,34)/SRC545. The same physical hop is FORK (pos=2) in the
// departing owner and TRANSIT (pos=1) in the surviving shared replica.
// Import the two recorded paths and replacement; a scaffold trunk owns their
// common first five fragments. Names are generated by Fixture::name.
void fanout_fork_role_regression() {
  Fixture f(80,80);
  Route prefix{f.pin({54,35},150,true),f.edge({54,35},{54,33},150,193,193,0),
      f.edge({54,33},{57,33},193,256,256,2),
      f.edge({57,33},{58,33},256,257,261,2),
      f.edge({58,33},{62,34},261,545,545,2)};
  for(auto &w:prefix) if(w.type==Wire::WIRE_CROSSBAR) w.owns_landing=true;
  Route trunk=prefix; f.terminal(trunk,{62,34},545,3000); f.install(771,trunk);
  for(auto &w:prefix) w.shared=true;
  Route survivor=prefix;
  auto shared=f.edge({62,34},{63,35},545,545,545);
  shared.shared=true; shared.owns_dst=false; survivor.push_back(shared);
  survivor.push_back(f.edge({63,35},{62,36},545,3617,3617));
  survivor.push_back(f.edge({62,36},{53,36},3617,3078,3079));
  survivor.push_back(f.edge({53,36},{53,38},3079,65,65));
  survivor.push_back(f.edge({53,38},{57,38},65,1026,1025));
  survivor.push_back(f.edge({57,38},{53,42},1025,3377,3377));
  survivor.push_back(f.edge({53,42},{44,42},3377,3078,3079));
  survivor.push_back(f.edge({44,42},{40,42},3079,3076,3073));
  survivor.push_back(f.edge({40,42},{36,38},3073,3538,3538));
  survivor.push_back(f.edge({36,38},{31,34},3538,3538,3538));
  survivor.push_back(f.edge({31,34},{31,33},3538,248,242));
  survivor.push_back(f.edge({31,33},{31,31},242,193,193));
  f.terminal(survivor,{31,31},193,81,40,55);
  auto &kept=f.install(771,survivor);
  Route owner=prefix;
  owner.push_back(f.edge({62,34},{63,35},545,545,545,2));
  owner.push_back(f.edge({63,35},{63,29},545,161,161));
  owner.push_back(f.edge({63,29},{67,33},161,817,817));
  owner.back().owns_landing=true;
  Wire gap; gap.type=Wire::WIRE_GAP; gap.from={67,33}; gap.to={63,34};
  gap.from_node_type=gap.to_node_type=fpga::CB_NODE_DST;
  gap.from_node=817; gap.to_node=3616; owner.push_back(gap);
  owner.push_back(f.edge({63,34},{62,33},3616,3809,3809));
  owner.push_back(f.edge({62,33},{63,32},3809,737,737));
  owner.push_back(f.edge({63,32},{63,30},737,193,193));
  owner.push_back(f.edge({63,30},{67,25},193,977,977));
  owner.push_back(f.edge({67,25},{66,25},977,3842,3847));
  owner.push_back(f.edge({66,25},{63,27},3847,3617,3617));
  owner.push_back(f.edge({63,27},{58,31},3617,3377,3377));
  owner.push_back(f.edge({58,31},{50,31},3377,3078,3079));
  owner.push_back(f.edge({50,31},{50,33},3079,65,65));
  owner.push_back(f.edge({50,33},{53,33},65,258,262));
  f.terminal(owner,{53,33},262,81,39,54);
  auto &replaced=f.install(771,owner);
  // Recorded replacement reaches the detached tail through a different DST.
  f.edge({54,35},{63,35},150,1027,1029,0);
  f.edge({63,35},{63,34},1029,242,246);
  f.edge({63,34},{62,33},246,3809,3809);
  f.rebuild();
  require(kept.size()==19 && replaced.size()==21,"fork-role replay shape changed");
  auto &net=f.signal(771).net;
  const auto before=f.state();
  require(!fpga::repairNetRouteGaps(net,2,[]{return true;}) && f.state()==before,
          "cancelled fork-role repair changed live state");
  require(fpga::repairNetRouteGaps(net,2),"source replacement did not complete");
  require(f.tile({62,34}).cb.src.jump.testBit(545),
          "BUG: FORK/TRANSIT metadata mismatch released a surviving shared SRC");
  require(!kept[5].shared && fpga::isRouteComplete(kept),
          "surviving transit did not inherit physical fork ownership");
  check_complete_path(f,replaced,12);
  f.state();
  require(fpga::unrouteNetRoute(net,2),"replacement teardown failed");
  require(f.tile({62,34}).cb.src.jump.testBit(545),"teardown lost surviving fork SRC");
  f.state();
  std::puts("fork/transit aliases retain their physical source lease across replacement");
}

// Dedicated endpoints can lease a resource tile pin beside the route CB.
// The audit must find that owner through endpoint metadata, not invent a leak.
void resource_pin_audit_regression() {
  Fixture f;
  Route route{f.pin({1,1},10,true),f.edge({1,1},{3,3},10,20,30,0)};
  f.terminal(route,{3,3},30,40);
  route.back().resource={4,3}; route.back().resource_node=8;
  auto &stored=f.install(763,std::move(route));
  auto &resource=f.tile({4,3});
  resource.pin_state.leased_nodes.setBit(40);
  auto &net=f.signal(763).net;
  for(bool indexed:{false,true}) {
    auto audit=[&] {
      std::ostringstream out;
      return indexed ? fpga::auditTileCongestion(resource,out,fpga::CongestionAuditIndex({&net})) :
          fpga::auditTileCongestion(resource,out,std::vector<rtl::Net *>{&net});
    };
    // Both authoritative design scans and their spatial index must see the
    // resource-side pin even though route registrations live on the CB tile.
    auto result=audit();
    require(result.leased_nodes==1 && result.orphan_leases==0 &&
        result.missing_registrations==0,"resource-side endpoint falsely reported as orphan lease");
    // A genuinely unrelated pin still has no owner; do not whitelist a tile.
    resource.pin_state.leased_nodes.setBit(41);
    require(audit().orphan_leases==1,"resource audit hid an unrelated leaked pin");
    resource.pin_state.leased_nodes.clearBit(41);
    // Removing the endpoint association must expose the now-unowned lease.
    stored.back().resource={-1,-1};
    require(audit().orphan_leases==1,"resource audit retained a stale endpoint claim");
    stored.back().resource={4,3};
    // Endpoint metadata does not mandate a second resource-side lease for
    // generic routing, whose pin is leased only on its attached crossbar.
    resource.pin_state.leased_nodes.clearBit(40);
    require(audit().missing_leases==0,"annotation alone required a resource-side lease");
    resource.pin_state.leased_nodes.setBit(40);
  }
}

void fanout_island_audit_regression() {
  Fixture f;
  Route a{f.pin({1,1},10,true),f.edge({1,1},{3,3},10,20,30,0)};
  f.terminal(a,{3,3},30,40);
  f.install(761,a);
  Route b{f.pin({2,2},11,true),f.edge({2,2},{3,3},11,21,30,0)};
  f.terminal(b,{3,3},30,41);
  f.install(762,b);
  auto &one=f.signal(761).net;
  auto &two=f.signal(762).net;
  auto audit=[&] {
    std::ostringstream out;
    fpga::auditTileCongestion(f.tile({3,3}),out,{&one,&two});
    return out.str();
  };
  // Diagnostic-only corruption fixture: both claims have live objects, valid
  // indices and a lease. They still cannot be called VERIFIED: two drivers
  // feed the same DST, just as after the captured island landing was lost.
  require(audit().find("status=CONFLICTING_DRIVERS")!=std::string::npos,
          "BUG: congestion audit calls two different drivers on one node VERIFIED");
  two.routes[0].from=one.routes[0].from;
  two.routes[0].from_port=one.routes[0].from_port;
  require(audit().find("status=CONFLICTING_DRIVERS")==std::string::npos,
          "audit rejects legitimate same-source aliases");
  two.routes[0].from_port="other_output";
  require(audit().find("status=CONFLICTING_DRIVERS")!=std::string::npos,
          "audit ignores different output ports on the same driver");
  // A borrowed incoming DST is required even though the FORK does not own
  // it. Corrupt the owner/lease deliberately to test the audit independently
  // of exactly the ownership flags that caused the real cleanup bug.
  f.signal(761).sinks[0].wires[0][2].owns_dst=false;
  f.signal(762).sinks[0].wires[0][2].owns_dst=false;
  f.tile({3,3}).cb.dst.jump.clearBit(30);
  require(audit().find("status=MISSING_LEASE")!=std::string::npos,
          "BUG: audit hides a missing borrowed incoming DST behind owns_dst=false");
  std::puts("island audit: incompatible drivers and missing borrowed DST detected");
}

void fanout_long_grounding_regression() {
  Fixture f(80,80);
  Route r{f.pin({45,38},154,true),f.edge({45,38},{45,39},154,17,20,0),
      f.edge({45,39},{45,41},20,65,65),f.edge({45,41},{41,41},65,3076,3073),
      f.edge({41,41},{45,37},3073,978,978),f.edge({45,37},{45,38},978,18,21)};
  f.terminal(r,{45,38},21,88,49,64);
  auto &route=f.install(162186,std::move(r));
  auto &s=f.signal(162186);
  f.rebuild();
  const std::vector<fpga::RouteCutNode> cuts{{{45,39},fpga::CB_NODE_SRC,65},
      {{45,41},fpga::CB_NODE_DST,65}};
  require(fpga::unrouteNetRoutesFromNodes(fpga::findNetRouteCutVictims(cuts),cuts,true),
          "could not reproduce the middle preemption");
  require(route.size()==7,"long Grounding replay lost requester fragments");
  check_gap(route[2],{45,39},20,{41,41},3073,fpga::CB_NODE_DST);
  Route takeoff{f.pin({45,39},154,true),f.edge({45,39},{45,41},154,65,65,0)};
  f.terminal(takeoff,{45,41},65,126);
  auto &protected_takeoff=f.install(163088,std::move(takeoff));
  auto gap=[&](Route &v,Coord a,int dst,Coord b,int next) {
    v.back().owns_landing=true;
    Wire w; w.type=Wire::WIRE_GAP; w.from=a; w.to=b;
    w.from_node_type=w.to_node_type=fpga::CB_NODE_DST;
    w.from_node=dst; w.to_node=next; v.push_back(w);
  };
  Route owner{f.pin({58,32},156,true),f.edge({58,32},{57,33},156,3619,3619,0),
      f.edge({57,33},{57,34},3619,19,22),f.edge({57,34},{58,35},22,547,547)};
  gap(owner,{58,35},547,{62,40},544);
  owner.push_back(f.edge({62,40},{66,40},544,1024,1024));
  owner.push_back(f.edge({66,40},{63,40},1024,3841,3846));
  owner.push_back(f.edge({63,40},{62,39},3846,3809,3809));
  owner.push_back(f.edge({62,39},{58,40},3809,246,3843));
  gap(owner,{58,40},3843,{49,39},3841);
  owner.push_back(f.edge({49,39},{45,38},3841,3810,3810));
  owner.push_back(f.edge({45,38},{41,38},3810,3076,3073));
  owner.push_back(f.edge({41,38},{41,37},3073,248,242));
  f.terminal(owner,{41,37},242,102,12,26);
  require(owner.size()==15,"long Grounding replay lost owner fragments");
  auto &victim=f.install(162283,std::move(owner));
  f.edge({45,38},{41,38},154,3076,3073,0);
  f.edge({41,38},{44,38},3073,258,262);
  f.edge({44,38},{45,38},262,259,257);
  auto &cb=*f.tile({45,38}).cb_type;
  for(int j:{26,12}) f.name({45,38},fpga::CB_NODE_JOINT,j);
  cb.dst_joint[257].joint.setBit(26);
  cb.joint_joint[26].joint.setBit(12);
  cb.joint_joint[12].joint.setBit(64);
  f.rebuild();
  const auto before=f.state();
  require(!fpga::repairNetRouteGaps(s.net,0) && f.state()==before,
          "free-only search crossed a foreign transit lease");
  pnr::RouteDesign router; router.fpga=&f.device; router.fanout_stage=true;
  pnr::RouteDesign::RouteTask task{.from=&s.driver,.to=&s.sinks[0],.net=&s.net,
      .from_port="out",.to_port="in",.net_name=s.net.routes[0].route_name,
      .fanout=true,.endpoints_prepared=true};
  router.route_preemption_enabled=false;
  require(!router.routeNetTask(task) && f.state()==before,"disabled preemption changed leases");
  router.route_preemption_enabled=true;
  f.signal(162283).net.route_protected=true;
  require(!router.routeNetTask(task) && f.state()==before,"long Grounding cut a protected owner");
  f.signal(162283).net.route_protected=false;
  for(const auto [a,b]:std::array<std::pair<int,int>,3>{{{26,12},{12,64},{64,49}}}) {
    cb.joint_joint[a].joint.clearBit(b);
    require(!router.routeNetTask(task) && f.state()==before && router.pending_route_todo.empty(),
            "long Grounding cut transit before proving the complete terminal chain");
    cb.joint_joint[a].joint.setBit(b);
  }
  require(router.routeNetTask(task) && fpga::isRouteComplete(route),
          "BUG: repair misses validated three-hop Takeoff/ Grounding with four terminal JOINTs");
  check_complete_path(f,route,3);
  check_complete_path(f,protected_takeoff,1);
  require(router.pending_route_todo.size()==1 &&
          router.pending_route_todo.front().net==&f.signal(162283).net,
          "long Grounding did not requeue exactly its transit owner");
  require(fpga::hasRouteGaps(victim) && victim.front().local==156 && victim.back().local==102,
          "long Grounding discarded the owner's retained endpoints");
  require(f.tile({45,38}).cb.joint.jump.testBit(26) &&
          f.tile({45,38}).cb.joint.jump.testBit(12) &&
          f.tile({45,38}).cb.joint.jump.testBit(64) &&
          f.tile({45,38}).cb.joint.jump.testBit(49),
          "long Grounding failed to register its four-JOINT terminal chain");
  f.state();
  std::printf("long Grounding: three hops, four terminal JOINTs, atomic transit cut, ownership clean\n");
}

// A remote sink lies outside the source's five-Tile Grounding window.
// Real failure: two preemptions left nine fragments and a trapped prefix.
// Retain that geometry, the 27-fragment transit owner and the six-hop bypass
// proved by the read-only live probe. Seven unrelated owner bindings and other
// blocked alternatives are omitted; shared ownership is normalized accordingly.
// All names are synthetic; all busy resources have registered route owners.
void fanout_remote_takeoff_regression(bool remote_blocked) {
  Fixture f(90,90);
  Route r{f.pin({54,37},163,true),f.edge({54,37},{54,38},163,20,24,0)};
  r.back().owns_landing=true;
  Wire gap; gap.type=Wire::WIRE_GAP; gap.from={54,38}; gap.to={57,41};
  gap.from_node_type=gap.to_node_type=fpga::CB_NODE_DST; gap.from_node=24; gap.to_node=547;
  r.push_back(gap);
  r.push_back(f.edge({57,41},{58,42},547,547,547));
  r.push_back(f.edge({58,42},{58,48},547,99,99));
  r.push_back(f.edge({58,48},{54,48},99,3079,3075));
  r.push_back(f.edge({54,48},{57,47},3075,262,260));
  f.terminal(r,{57,47},260,88,36,51);
  require(r.size()==9,"remote Takeoff snapshot lost requester fragments");
  auto &route=f.install(162349,std::move(r));
  auto &s=f.signal(162349);
  // The old prefix cannot continue: its next physical hop is a newer Takeoff.
  Route newer{f.pin({54,38},154,true),f.edge({54,38},{54,39},154,21,25,0)};
  f.terminal(newer,{54,39},25,95);
  auto &protected_route=f.install(7001,std::move(newer));
  f.edge({54,38},{54,39},24,21,25);

  Route owner{f.pin({53,40},148,true),
      f.edge({53,40},{62,40},148,1031,1031,0),
      f.edge({62,40},{62,39},1031,244,248),
      f.edge({62,39},{62,38},248,244,248),
      f.edge({62,38},{63,38},248,261,258),
      f.edge({63,38},{66,39},258,546,546),
      f.edge({66,39},{70,43},546,818,818),
      f.edge({70,43},{71,43},818,259,257),
      f.edge({71,43},{75,44},257,545,545),
      f.edge({75,44},{75,46},545,65,65),
      f.edge({75,46},{75,53},65,97,97),
      f.edge({75,53},{70,48},97,3538,3538),
      f.edge({70,48},{75,44},3538,978,978),
      f.edge({75,44},{75,38},978,162,162),
      f.edge({75,38},{76,38},162,259,257),
      f.edge({76,38},{81,38},257,256,256),
      f.edge({81,38},{81,40},256,64,64),
      f.edge({81,40},{70,40},64,3073,3078),
      f.edge({70,40},{62,40},3078,3073,3078),
      f.edge({62,40},{58,41},3078,3616,3616),
      f.edge({58,41},{54,37},3616,3537,3537),
      f.edge({54,37},{50,41},3537,3376,3376),
      f.edge({50,41},{53,41},3376,257,261),
      f.edge({53,41},{53,42},261,17,20),
      f.edge({53,42},{54,42},20,258,262)};
  f.terminal(owner,{54,42},262,95,11,25);
  require(owner.size()==27,"remote Takeoff snapshot lost transit owner fragments");
  auto &victim=f.install(166839,std::move(owner));
  // Exact six-hop replacement from the live probe: only the first SRC and
  // landing require a cut. The intermediate LOCAL bridge is routing topology.
  f.edge({54,37},{50,41},163,3376,3376,0);
  f.name({50,41},fpga::CB_NODE_LOCAL,183);
  f.tile({50,41}).cb_type->dst_local[3376].local.setBit(183);
  f.edge({50,41},{54,45},183,819,819,0);
  f.edge({54,45},{53,46},819,3619,3619);
  f.edge({53,46},{53,47},3619,19,22);
  f.edge({53,47},{54,48},22,547,547);
  f.edge({54,48},{57,47},547,262,260);
  if(remote_blocked) {
    // An occupied SECOND hop is transit, not Takeoff, even though its input
    // uses an intermediate LOCAL bridge. It is also outside Grounding.
    Route middle{f.pin({49,41},159,true),f.edge({49,41},{50,41},159,900,910,0),
        f.edge({50,41},{54,45},910,819,819)};
    f.terminal(middle,{54,45},819,96);
    f.install(7002,std::move(middle));
  }
  f.rebuild();
  const auto before=f.state();
  require(!fpga::repairNetRouteGaps(s.net,0) && f.state()==before,
          "free-only repair crossed a transit owner");
  pnr::RouteDesign router; router.fpga=&f.device; router.fanout_stage=true;
  pnr::RouteDesign::RouteTask task{.from=&s.driver,.to=&s.sinks[0],.net=&s.net,
      .from_port="out",.to_port="in",.net_name=s.net.routes[0].route_name,
      .fanout=true,.endpoints_prepared=true};
  router.route_preemption_enabled=false;
  require(!router.routeNetTask(task) && f.state()==before,"disabled preemption changed leases");
  router.route_preemption_enabled=true;
  f.signal(166839).net.route_protected=true;
  require(!router.routeNetTask(task) && f.state()==before,"remote Takeoff cut protected owner");
  f.signal(166839).net.route_protected=false;
  auto &bridge=f.tile({50,41}).cb_type->dst_local[3376].local;
  bridge.clearBit(183);
  require(!router.routeNetTask(task) && f.state()==before && router.pending_route_todo.empty(),
          "remote Takeoff cut transit before validating a complete path");
  bridge.setBit(183);
  if(remote_blocked) {
    require(!router.routeNetTask(task) && f.state()==before && router.pending_route_todo.empty(),
            "repair preempted remote transit after its initial Takeoff");
    return;
  }
  require(router.routeNetTask(task) && fpga::isRouteComplete(route),
          "BUG: repair misses source Takeoff outside the Grounding window");
  check_complete_path(f,route,6);
  check_complete_path(f,protected_route,1);
  require(router.pending_route_todo.size()==1 &&
          router.pending_route_todo.front().net==&f.signal(166839).net,
          "remote Takeoff failed to requeue exactly the affected transit owner");
  require(fpga::hasRouteGaps(victim) && victim.front().local==148 && victim.back().local==95,
          "remote Takeoff discarded the transit owner's retained endpoints");
  f.state();
}

// A failed bound task must be audited as a repair, not as a new connection.
// Its private terminal lease is reusable and must remain untouched by probes.
void fanout_retained_audit_regression() {
  Fixture f(10,10);
  Route r{f.pin({1,1},3,true),f.edge({1,1},{2,1},3,10,20,0)};
  r.back().owns_landing=true;
  Wire gap;
  gap.type=Wire::WIRE_GAP; gap.from={2,1}; gap.to={4,1};
  gap.from_node_type=gap.to_node_type=fpga::CB_NODE_DST;
  gap.from_node=20; gap.to_node=21;
  r.push_back(gap);
  f.terminal(r,{4,1},21,4,7);
  auto &route=f.install(701,std::move(r));
  auto &s=f.signal(701);
  f.edge({2,1},{4,1},20,11,22);
  auto &cb=*f.tile({4,1}).cb_type;
  cb.dst_joint[22].joint.setBit(7);
  f.rebuild();
  const auto before=f.state();
  std::ostringstream trace;
  // A real free repair exists despite the already-leased private terminal.
  require(fpga::auditNetRouteReachability(s.net,0,trace),"retained audit fixture has no free path");
  // The old caller lost private-tail permissions by passing only endpoints.
  require(!fpga::auditUnroutedReachability(s.net,s.net.routes[0],route.front(),route.back(),
      trace,64),"fixture does not distinguish bound and unbound ownership");
  trace.str(""); trace.clear();
  // Reproduce failRouting's completion probe with an existing binding.
  require(fpga::auditRouteCompletion(s.net,s.net.routes[0],route.front(),route.back(),
      trace,{},64,64,8),"BUG: failed-task completion audit silently skips a retained route");
  require(trace.str().find("SOURCE_DIAGNOSTIC_COMPLETE")!=std::string::npos,
      "retained completion audit did not print a witness");
  // Neither the positive probe nor its owner lookups may repair or unlease anything.
  require(f.state()==before && fpga::hasRouteGaps(route),"read-only audit mutated retained routing");
  trace.str(""); trace.clear();
  // Exhaust the small captures and require the larger probe to retain its
  // replay edges too, without committing the retained route's repair.
  require(fpga::auditRouteCompletion(s.net,s.net.routes[0],route.front(),route.back(),
      trace,{},1,64,8,true),"larger captured audit failed to find the free repair");
  const auto larger=trace.str().rfind("SOURCE_SEARCH begin");
  require(larger!=std::string::npos &&
      trace.str().find("SOURCE_SEARCH_EDGE ",larger)!=std::string::npos &&
      trace.str().find("SOURCE_SEARCH_NODE ",larger)!=std::string::npos,
      "larger completion probe omitted the frontier needed for replay");
  require(f.state()==before,"larger captured audit mutated routing");
  auto wrong_sink=route.back(); ++wrong_sink.local;
  // The retained binding must not authorize a different candidate endpoint.
  require(!fpga::auditRouteCompletion(s.net,s.net.routes[0],route.front(),wrong_sink,
      trace,{},64,64,8),"retained audit accepted a different physical sink");
  require(f.state()==before,"mismatched endpoint audit changed routing");
  cb.dst_joint[22].joint.clearBit(7);
  trace.str(""); trace.clear();
  // A genuinely disconnected terminal must fail, still with an actual search trace.
  require(!fpga::auditRouteCompletion(s.net,s.net.routes[0],route.front(),route.back(),
      trace,{},64,64,8),"disconnected retained audit succeeded");
  require(trace.str().find("SOURCE_SEARCH begin")!=std::string::npos,
      "negative retained audit silently skipped its search");
  require(f.state()==before,"negative audit changed leases or bindings");
  cb.dst_joint[22].joint.setBit(7);
  // The diagnostic witness must agree with a real production repair.
  require(fpga::repairNetRouteGaps(s.net,0,{},64,8) && fpga::isRouteComplete(route),
      "production repair disagrees with its diagnostic witness");
  f.state();
}

// Captured free frontier of a retained fanout repair. Keep numeric topology
// and the owned tail, but replace every external resource/cell name.
// Production exhausted 32768 states; removing unrelated chip topology reduces
// this replay to 32527. A 30000-state bound reproduces the same starvation of
// a free reconnection at (54,109), well before the final pin at (58,89).
// The wide case captures the next failure: 65536 states were insufficient for
// (45,197)/LOCAL146 -> (57,51)/LOCAL97. Its real completion used 155801 states.
// Preserve the free frontier and retained two-joint terminal, with real owners
// for every installed lease, then check completion and full lease teardown.
void fanout_retained_frontier_regression(bool bounded_failure,bool wide) {
  Fixture f(wide?208:207,144);
  f.device.cb_types.reserve(wide?2048:1024);
  const Coord source=wide?Coord{45,197}:Coord{82,162};
  const Coord sink=wide?Coord{57,51}:Coord{58,89};
  const int source_local=wide?146:142, sink_local=wide?97:84;
  using Node=std::tuple<int,int,int,int>;
  auto arc=[&](Node a,Node b) { f.arc(a,b); };
  const auto directory=std::filesystem::path(__FILE__).parent_path();
  const std::string prefix=wide?"fanout_retained_wide":"fanout_retained";
  f.loadFrontier(prefix+"_frontier.txt",wide?706542:105671,wide);
  int x,y,nx,ny;
  std::ifstream saved(directory/(prefix+"_route.txt"));
  require(bool(saved),"missing retained binding snapshot");
  Route route; int type,local,src,dst,joint,joint2,pos,fkind,fid,tkind,tid;
  while(saved>>type>>x>>y>>nx>>ny>>local>>src>>dst>>joint>>joint2>>pos>>fkind>>fid>>tkind>>tid) {
    Wire w; w.type=static_cast<decltype(w.type)>(type); w.from={x,y}; w.to={nx,ny};
    w.local=local; w.jump=src; w.dst=dst; w.joint=joint; w.joint2=joint2;
    w.pos=pos==2?1:pos; w.owns_dst=true; w.shared=false;
    w.from_node_type=static_cast<fpga::CBNodeNameType>(fkind); w.from_node=fid;
    w.to_node_type=static_cast<fpga::CBNodeNameType>(tkind); w.to_node=tid;
    if(w.type==Wire::WIRE_TILE_PIN) w=f.pin({x,y},local,route.empty());
    else if(w.type==Wire::WIRE_CROSSBAR) {
      Node previous{x,y,w.pos==0?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST,local};
      auto add=[&](Node next) {arc(previous,next);previous=next;};
      if(joint2>=0) add({x,y,fpga::CB_NODE_JOINT,joint2});
      if(joint>=0) add({x,y,fpga::CB_NODE_JOINT,joint});
      if(src>=0) {
        add({x,y,fpga::CB_NODE_SRC,src}); add({nx,ny,fpga::CB_NODE_DST,dst});
        w.owns_landing=true;
      } else add({x,y,fpga::CB_NODE_LOCAL,sink_local});
    }
    else require(w.type==Wire::WIRE_GAP,"unexpected retained fragment");
    route.push_back(w);
  }
  require(saved.eof() && route.size()==(wide?69:39),"incomplete retained binding snapshot");
  auto &installed=f.install(712,std::move(route));
  // Output pins are not input-pin leases in production.
  f.tile(source).pin_state.leased_nodes.clearBit(source_local);
  f.rebuild(); f.device.rebuildIncomingDstMasks();
  const auto before=f.state();
  auto &net=f.signal(712).net;
  SourceRepairTrace trace(net.name);
  const size_t budget=bounded_failure?30000:fpga::DEFAULT_ROUTE_REPAIR_STATES;
  // Preserve the known capacity failure as evidence, not as a claimed fix.
  // The larger-budget control must find a real path without preempting anyone.
  const bool done=bounded_failure?fpga::repairNetRouteGaps(net,0,{},budget,200):
      fpga::repairNetRouteGaps(net,0);
  std::ifstream log(trace.path);
  for(std::string line;std::getline(log,line);)
    if(line.starts_with("SOURCE_SEARCH begin") || line.starts_with("SOURCE_SEARCH end") ||
        line.starts_with("SOURCE_SEARCH_SUFFIX")) std::cout << line << '\n';
  f.state();
  if(bounded_failure && !done) {
    require(f.state()==before,"failed bounded search changed leases or retained binding");
    std::ostringstream witness;
    // The corrected diagnostic must distinguish budget exhaustion from no path.
    require(fpga::auditRouteCompletion(net,net.routes[0],installed.front(),installed.back(),
        witness,{},40000,40000,200),"capacity failure has no independently verified completion");
    require(f.state()==before,"completion witness changed retained state");
    return;
  }
  require(done,"recorded free completion failed at its positive-control budget");
  // Completion must be a real route with the original physical endpoints.
  require(fpga::isRouteComplete(installed) && same(installed.front().from,source) &&
      same(installed.back().from,sink),"repair moved endpoints or retained a gap");
  // Removing the repaired binding must free all its leases and registrations.
  require(fpga::unrouteNetRoute(net,0),"cannot remove repaired binding");
  f.state();
}

// Dense reconvergence repeats the same already-dominated exits from many
// arrivals. Reject those labels before walking their entire ancestor chains.
// Operation counts make this a deterministic performance regression, not a
// machine-dependent deadline; real foreign occupancy must remain protected.
void source_dominance_work_regression() {
  constexpr int length=48;
  std::map<int,std::string> paths;
  for(bool exhaustive_trace:{false,true}) for(int lanes:{1,8,32}) {
    Fixture f(4,length+2);
    const Coord start{1,1}, finish{length,1};
    for(int x=1;x<length;++x) for(int lane=0;lane<lanes;++lane) {
      if(x==1) f.edge({x,1},{x+1,1},144,200+lane,500+lane,0);
      else for(int incoming=0;incoming<lanes;++incoming)
        f.edge({x,1},{x+1,1},500+incoming,200+lane,500+lane);
    }
    for(int lane=0;lane<lanes;++lane)
      f.arc({length,1,fpga::CB_NODE_DST,500+lane},{length,1,fpga::CB_NODE_LOCAL,91});
    // The wide cases must select a different arrival at this real busy wire.
    if(lanes>1) {
      Route busy{f.pin({24,2},3990,true),f.edge({24,2},{24,1},3990,3990,500,0)};
      f.terminal(busy,{24,1},500,3990); f.install(714,std::move(busy));
      f.tile({24,2}).pin_state.leased_nodes.clearBit(3990);
    }
    const Wire from=f.pin(start,144,true), to=f.pin(finish,91,false);
    auto &s=f.signal(713);
    s.driver.tile.set(&f.device.tile_grid[start.y*f.device.size_width+start.x]);
    s.sinks.emplace_back(); s.sink_cells.emplace_back();
    auto &sink=s.sinks.back();
    s.sink_cells.back().name="receiver_713"; s.sink_cells.back().type="logic_731";
    sink.cell_ref.set(&s.sink_cells.back());
    sink.tile.set(&f.device.tile_grid[finish.y*f.device.size_width+finish.x]);
    rtl::NetRouteBinding endpoints;
    endpoints.from=&s.driver; endpoints.to=&sink;
    endpoints.from_port="out"; endpoints.to_port="in"; endpoints.route_name="branch_713";
    f.rebuild(); f.device.rebuildIncomingDstMasks();
    const auto before=f.state();
    auto foreign_state=[&] {
      std::ostringstream out;
      if(lanes>1) fpga::dumpNetRouteHistory(f.signal(714).net,out);
      return out.str();
    };
    const auto original_owner=foreign_state();
    // Even a dense frontier must not leak speculative leases on cancellation.
    unsigned polls=0;
    require(!fpga::routeUnboundNetFromSource(s.net,endpoints,from,to,[&]{return ++polls>100;},32768),
            "cancelled dense search unexpectedly completed");
    require(f.state()==before && sink.wires.empty(),"cancelled dense search mutated ownership");
    std::string decisions;
    std::unique_ptr<SourceRepairTrace> frontier;
    if(exhaustive_trace) frontier=std::make_unique<SourceRepairTrace>(s.net.name);
    {
      RepairDecisionTrace trace;
      require(fpga::routeUnboundNetFromSource(s.net,endpoints,from,to,{},32768),
              "dense reconvergent fabric failed to route");
      decisions=trace.read();
    }
    std::cout << "dense source lanes=" << lanes << " exhaustive_trace=" << exhaustive_trace << '\n' << decisions;
    // Check completion and teardown before checking the work bound.
    require(s.net.routes.size()==1 && sink.wires.size()==1 &&
        fpga::isRouteComplete(sink.wires.front()),"dense search left a partial route");
    check_complete_path(f,sink.wires.front(),length-1);
    // Diagnostics must not change selection or drop eligible replay edges.
    std::ostringstream path;
    for(const auto &w:sink.wires.front())
      path << w.type << ' ' << w.from.x << ' ' << w.from.y << ' ' << w.to.x << ' ' << w.to.y
           << ' ' << w.local << ' ' << w.jump << ' ' << w.dst << ' ' << w.joint << '\n';
    if(exhaustive_trace) {
      require(paths.at(lanes)==path.str(),"diagnostics changed the chosen dense route");
      std::ifstream log(frontier->path); std::string line; size_t edges=0;
      while(std::getline(log,line)) if(line.starts_with("SOURCE_SEARCH_EDGE ")) ++edges;
      require(edges>=size_t(lanes*lanes*(length-3)),"diagnostics dropped dominated replay edges");
    } else paths[lanes]=path.str();
    f.state();
    require(fpga::unrouteNetRoute(s.net,0),"dense requester did not unroute");
    // Empty binding metadata survives unrouting; audit leases, not that metadata.
    f.state();
    require(sink.wires.front().empty() && foreign_state()==original_owner,
            "dense requester changed another route or retained its fragments");
    size_t states=0,eligibility=0,parents=0,rejected=0;
    std::istringstream lines(decisions); std::string line;
    while(std::getline(lines,line)) {
      size_t n,limit,e,p,d; int reached,hops;
      if(std::sscanf(line.c_str(),"REPAIR source states=%zu limit=%zu reached=%d hops=%d eligibility_checks=%zu parent_checks=%zu dominated=%zu",
          &n,&limit,&reached,&hops,&e,&p,&d)==7) {
        states+=n; eligibility+=e; parents+=p; rejected+=d;
      }
    }
    require(states>0,"missing dense search work counters");
    require(lanes==1 || rejected>states,"fixture did not exercise dominated reconvergence");
    // At most a few non-dominated checks per label; not lanes checks each.
    require(exhaustive_trace || eligibility<=4*states,"dominated exits repeated eligibility work");
    require(parents<=states*2*length,"dominated exits repeated whole-path ancestry walks");
  }
}

// Chunk 4 repeatedly probes the same occupied resources from different edges.
// Use real neutral-named transit owners; every attempt must be read-only and
// a new attempt after an unroute must not inherit the previous owner's answer.
void docking_cut_checks_regression() {
  Fixture f(8,8);
  std::vector<fpga::RouteCutNode> nodes;
  for(int n=0;n<8;++n) {
    Route owner{f.pin({1,2},100+n,true),f.edge({1,2},{2,2},100+n,200+n,300+n,0),
        f.edge({2,2},{3,2},300+n,400+n,500+n)};
    f.terminal(owner,{3,2},500+n,600+n);
    f.install(810+n,std::move(owner));
    // Outputs own a CB LOCAL, not an input pin reservation in production.
    f.tile({1,2}).pin_state.leased_nodes.clearBit(100+n);
    nodes.push_back({{2,2},fpga::CB_NODE_SRC,400+n});
    nodes.push_back({{2,2},fpga::CB_NODE_DST,300+n});
  }
  f.rebuild(); f.state();
  for(int phase=0;phase<3;++phase) {
    const auto before=f.state();
    size_t queries=0,requests=0;
    auto check=[&](const fpga::RouteCutNode &cut) {
      ++queries;
      auto &tile=f.tile(cut.tile);
      return fpga::allNetOwnersMatch(tile,cut.type,cut.node,[&](const auto &owner) {
        // The last independent source is protected in this attempt.
        return owner.net!=&f.signal(817).net;
      });
    };
    pnr::DockingCutChecks cached(check);
    std::set<std::tuple<int,int,int,int>> queried;
    // Repeat each positive and negative result, including mixed cut lists.
    for(int repeat=0;repeat<128;++repeat) for(size_t i=0;i<nodes.size();++i) {
      std::vector<fpga::RouteCutNode> cuts{nodes[i],nodes[(i+3)%nodes.size()]};
      bool expected=true;
      for(const auto &cut:cuts) {
        queried.emplace(cut.tile.x,cut.tile.y,cut.type,cut.node);
        const int owner=cut.type==fpga::CB_NODE_SRC?cut.node-400:cut.node-300;
        if(!same(cut.tile,{2,2}) || (cut.type!=fpga::CB_NODE_SRC && cut.type!=fpga::CB_NODE_DST)
            || owner==7 || (phase>0 && owner==0)) { expected=false; break; }
      }
      ++requests;
      // No orphan lease or protected owner can become eligible via caching.
      require(cached.eligible(cuts)==expected,"cached cut policy changed an owner decision");
    }
    require(cached.eligible({}),"empty cut list must remain eligible");
    std::cout << "docking cut checks phase=" << phase << " requests=" << requests
              << " queries=" << queries << " unique=" << queried.size() << '\n';
    // Cost depends on unique physical nodes, not candidate-edge fan-in.
    require(queries==queried.size(),"repeated docking edges rescanned unchanged owners");
    require(f.state()==before,"eligibility preview changed route leases or registrations");
    if(phase==0) {
      // A fresh search must see that the owner disappeared, not cached true.
      require(fpga::unrouteNet(f.signal(810).net),"cannot remove original transit owner");
      f.state();
    }
    if(phase==1) {
      // The same IDs on another tile/type are distinct cache keys.
      f.tile({4,2});
      nodes.push_back({{4,2},fpga::CB_NODE_SRC,400});
      nodes.push_back({{2,2},fpga::CB_NODE_JOINT,400});
    }
  }
}

// Free-only and preemption completion see identical leases in one unbound
// request. The second phase must reuse the exhausted free frontier, not walk
// the whole source tree again. Repeat with different corridor lengths and a
// real transit obstruction; failed/cancelled probes cannot alter any owner.
void source_frontier_reuse_regression() {
  for(int length:{8,24,48}) for(bool allow_cut:{false,true}) {
    Fixture f(5,length+2);
    Route seed{f.pin({1,1},144,true),f.edge({1,1},{2,1},144,200,500,0)};
    seed.push_back(f.edge({2,1},{2,2},500,201,501));
    f.terminal(seed,{2,2},501,90);
    f.install(713,std::move(seed));
    f.tile({1,1}).pin_state.leased_nodes.clearBit(144);
    for(int x=2;x<length;++x) f.edge({x,1},{x+1,1},500,200,500);
    f.arc({length,1,fpga::CB_NODE_DST,500},{length,1,fpga::CB_NODE_LOCAL,91});
    Route victim{f.pin({length,3},3990,true),
        f.edge({length,3},{length,2},3990,3990,3990,0),
        f.edge({length,2},{length,1},3990,3990,500),
        f.edge({length,1},{length,0},500,3990,3990)};
    f.terminal(victim,{length,0},3990,3990);
    f.install(714,std::move(victim));
    f.tile({length,3}).pin_state.leased_nodes.clearBit(3990);
    auto &s=f.signal(713);
    s.sinks.emplace_back(); s.sink_cells.emplace_back();
    auto &sink=s.sinks.back();
    s.sink_cells.back().name="receiver_713"; s.sink_cells.back().type="logic_731";
    sink.cell_ref.set(&s.sink_cells.back());
    sink.tile.set(&f.device.tile_grid[length+f.device.size_width]);
    rtl::NetRouteBinding binding;
    binding.from=&s.driver; binding.to=&sink;
    binding.from_port="out"; binding.to_port="in"; binding.route_name="branch_713_extra";
    const auto from=f.pin({1,1},144,true), to=f.pin({length,1},91,false);
    f.rebuild(); f.device.rebuildIncomingDstMasks();
    const auto before=f.state();
    int commits=0;
    auto preempt=[&](const std::vector<fpga::RouteCutNode> &cuts,bool commit,
                      std::vector<fpga::RouteCutNode> &released) {
      if(!allow_cut) return false;
      for(const auto &cut:cuts)
        if(!same(cut.tile,{length,1}) || cut.type!=fpga::CB_NODE_DST || cut.node!=500) return false;
      const auto victims=fpga::findNetRouteCutVictims(cuts);
      if(victims.empty()) return false;
      released=fpga::previewNetRouteCuts(victims,cuts,true);
      if(!commit) return !released.empty();
      ++commits;
      return fpga::unrouteNetRoutesFromNodes(victims,cuts,true);
    };
    require(!fpga::routeUnboundNetFromSource(s.net,binding,from,to,[]{return true;},4096,100,preempt),
            "cancelled frontier reuse committed a route");
    require(f.state()==before,"cancelled frontier reuse changed leases");
    {
      const char *key="SCALEPNR_FREE_SEARCH_STATES";
      const bool present=std::getenv(key)!=nullptr;
      const std::string original=present?std::getenv(key):"";
      struct Restore {
        const char *key; bool present; std::string value;
        ~Restore() { if(present) setenv(key,value.c_str(),1); else unsetenv(key); }
      } restore{key,present,original};
      setenv(key,"4",1);
      RepairDecisionTrace trace;
      // A small free-search cap must fail without allocating a route or
      // clearing the retained trunk/victim. The normal retry below must work.
      require(!fpga::routeUnboundNetFromSource(s.net,binding,from,to,{},4096,100),
              "free-source search ignored its smaller state cap");
      require(f.state()==before && sink.wires.empty(),"capped search changed routing state");
      require(trace.read().find("states=4 limit=4")!=std::string::npos,
              "free-source search did not report the effective cap");
    }
    std::string decisions;
    bool done=false;
    {
      RepairDecisionTrace trace;
      done=fpga::routeUnboundNetFromSource(s.net,binding,from,to,{},4096,100,preempt);
      decisions=trace.read();
    }
    std::cout << "source frontier length=" << length << " cut=" << allow_cut << '\n' << decisions;
    require(done==allow_cut,"frontier reuse changed terminal cut policy");
    f.state();
    require(commits==int(allow_cut),"search committed a preview or missed the winning cut");
    require(allow_cut ? fpga::isRouteComplete(sink.wires.front()) : f.state()==before,
            "failed reuse leaked leases or successful reuse left a partial route");
    // Reused search labels must reconstruct the same connected corridor.
    if(allow_cut) check_complete_path(f,sink.wires.front(),length-1);
    size_t reused=0,at=0;
    while((at=decisions.find("reused_frontier=1",at))!=std::string::npos) {++reused;++at;}
    // One completion succeeds immediately; two failed completion variants
    // must each consume their own retained/free-takeoff search, not rescan it.
    require(reused==(allow_cut?1u:2u),"preemption completion repeated the free source search");
  }
}

// Example 1's Moving timeout spent hundreds of seconds in GAP source searches.
// The early free probe and the later Grounding fallback see identical leases:
// an exact-boundary search between them is read-only. Reproduce that ordering
// with a long free corridor ending at a genuinely owned transit obstruction.
// This isolates repeated work, rather than pretending to copy the whole chip.

// The recorded fixed-endpoint recovery repeated three completed connections
// thousands of times. Its short approach takes two other signals' transit
// resources, although this exact 65-node, resolved-LOCAL bypass is free.
// Preserve those paths and real owners; add a wide irrelevant free branch
// to scale the production label pressure down to a deterministic small test.
// All resource/instance names come from the abstract fixture.
void moving_fixed_local_cycle_regression() {
  Fixture f(32,86);
  using Node=Fixture::Node;
  const std::vector<Node> free_path{
      {57,19,0,149},
      {57,19,3,1024},
      {62,19,4,1024},
      {62,19,3,544},
      {63,20,4,544},
      {63,20,3,1025},
      {71,20,4,1028},
      {71,20,3,544},
      {75,21,4,544},
      {75,21,3,3376},
      {70,25,4,3376},
      {70,25,3,1025},
      {81,25,4,1028},
      {81,25,3,3841},
      {76,25,4,3846},
      {76,25,0,30},
      {77,25,0,80},
      {78,26,0,361},
      {78,26,0,298},
      {78,26,0,337},
      {78,26,0,273},
      {78,26,0,113},
      {77,26,0,1},
      {76,26,0,1},
      {75,26,0,37},
      {74,26,0,1},
      {73,26,0,1},
      {72,26,0,1},
      {71,26,0,1},
      {70,26,0,37},
      {69,26,0,1},
      {68,26,0,1},
      {67,26,0,1},
      {66,26,0,37},
      {65,26,0,1},
      {64,26,0,1},
      {63,26,0,1},
      {62,26,0,37},
      {61,26,0,1},
      {60,26,0,1},
      {59,26,0,1},
      {58,26,0,1},
      {57,26,0,37},
      {56,26,0,1},
      {55,26,0,1},
      {54,26,0,1},
      {54,26,0,31},
      {54,25,0,66},
      {54,24,0,66},
      {54,23,0,66},
      {54,22,0,66},
      {54,21,0,66},
      {54,20,0,66},
      {54,19,0,66},
      {54,18,0,66},
      {54,17,0,66},
      {54,16,0,66},
      {54,15,0,66},
      {54,14,0,66},
      {54,13,0,66},
      {54,12,0,66},
      {54,11,0,66},
      {54,11,0,78},
      {54,11,1,29},
      {54,11,0,7},
  };
  const std::vector<Node> short_path{
      {57,19,0,149},
      {57,19,3,96},
      {57,25,4,96},
      {57,25,3,544},
      {58,27,4,544},
      {58,27,3,241},
      {58,25,4,245},
      {58,25,3,192},
      {58,23,4,192},
      {58,23,3,3808},
      {57,22,4,3808},
      {57,22,3,3072},
      {49,22,4,3077},
      {49,22,3,160},
      {49,16,4,160},
      {49,16,3,160},
      {49,10,4,160},
      {49,10,3,240},
      {50,11,4,259},
      {50,11,3,1030},
      {54,11,4,1027},
      {54,11,1,29},
      {54,11,0,7},
  };
  auto topology=[&](const std::vector<Node> &nodes) {
    for(size_t i=1;i<nodes.size();++i) {
      auto [x,y,kind,id]=nodes[i-1]; auto [nx,ny,nkind,nid]=nodes[i];
      if(kind==fpga::CB_NODE_LOCAL && nkind==fpga::CB_NODE_LOCAL) {
        f.name({x,y},fpga::CB_NODE_LOCAL,id); f.name({nx,ny},fpga::CB_NODE_LOCAL,nid);
        fpga::CBType::ResolvedLocal link; link.delta={nx-x,ny-y};
        link.target_cb_type_id=f.tile({nx,ny}).cb_type->type_id;
        link.target_node_type=fpga::CB_NODE_LOCAL; link.target_nodes.setBit(nid);
        f.tile({x,y}).cb_type->local_by_local[id].push_back(link);
      } else f.arc(nodes[i-1],nodes[i]);
    }
  };
  topology(free_path); topology(short_path);
  // Import the interrupted route and its two completed competitors, including
  // the second competitor's internal LOCAL bridge. No fabricated busy bits.
  auto lease=[&](Coord at,int kind,int node) {
    auto &cb=f.tile(at).cb;
    switch(kind) {
      case fpga::CB_NODE_LOCAL: cb.local.local.setBit(node); break;
      case fpga::CB_NODE_JOINT: cb.joint.jump.setBit(node); break;
      case fpga::CB_NODE_SRC: cb.src.jump.setBit(node); break;
      case fpga::CB_NODE_DST: cb.dst.jump.setBit(node); break;
    }
  };
  auto install=[&](int id,Route route) -> Route& {
    for(auto &w:route) {
      if(w.type==Wire::WIRE_CROSSBAR && w.jump>=0) w.owns_landing=true;
      if(w.type==Wire::WIRE_ROUTE_EDGE) {
        lease(w.from,w.from_node_type,w.from_node);
        lease(w.to,w.to_node_type,w.to_node);
      }
    }
    auto &out=f.install(id,std::move(route));
    f.tile(out.front().from).pin_state.leased_nodes.clearBit(out.front().local);
    return out;
  };
  Route route_730;
  route_730.push_back(f.pin({57,19},149,true));
  route_730.push_back(f.edge({57,19},{57,25},149,96,96,0));
  route_730.push_back(f.edge({57,25},{58,27},96,544,544,1));
  route_730.push_back(f.edge({58,27},{58,25},544,241,245,1));
  route_730.push_back(f.edge({58,25},{58,23},245,192,192,1));
  route_730.push_back(f.edge({58,23},{57,22},192,3808,3808,1));
  route_730.push_back(f.edge({57,22},{49,22},3808,3072,3077,1));
  route_730.push_back(f.edge({49,22},{49,16},3077,160,160,1));
  route_730.push_back(f.edge({49,16},{49,10},160,160,160,1));
  { Wire w; w.type=Wire::WIRE_GAP; w.from={49,10}; w.to={54,11};
    w.from_node_type=4; w.from_node=160; w.to_node_type=0; w.to_node=7; route_730.push_back(w); }
  route_730.push_back(f.pin({54,11},7,false));
  auto &installed_730=install(730,std::move(route_730));
  Route route_731;
  route_731.push_back(f.pin({53,14},159,true));
  route_731.push_back(f.edge({53,14},{49,10},159,3536,3536,0));
  route_731.push_back(f.edge({49,10},{50,11},3536,240,259,1));
  route_731.push_back(f.edge({50,11},{50,10},259,244,248,1));
  f.terminal(route_731,{50,10},248,119,-1,-1); route_731.pop_back();
  route_731.push_back(f.pin({50,10},119,false));
  auto &installed_731=install(731,std::move(route_731));
  Route route_732;
  route_732.push_back(f.pin({45,7},150,true));
  route_732.push_back(f.edge({45,7},{37,7},150,3073,3078,0));
  route_732.push_back(f.edge({37,7},{32,11},3078,3376,3376,1));
  { Wire w; w.type=Wire::WIRE_ROUTE_EDGE; w.from={32,11}; w.to={32,11};
    w.from_node_type=4; w.from_node=3376; w.to_node_type=0; w.to_node=183; route_732.push_back(w); }
  f.arc({32,11,4,3376},{32,11,0,183});
  { Wire w; w.type=Wire::WIRE_ROUTE_EDGE; w.from={32,11}; w.to={32,11};
    w.from_node_type=0; w.from_node=183; w.to_node_type=3; w.to_node=1031; route_732.push_back(w); }
  f.arc({32,11,0,183},{32,11,3,1031});
  { Wire w; w.type=Wire::WIRE_ROUTE_EDGE; w.from={32,11}; w.to={41,11};
    w.from_node_type=3; w.from_node=1031; w.to_node_type=4; w.to_node=1031; route_732.push_back(w); }
  f.arc({32,11,3,1031},{41,11,4,1031});
  route_732.push_back(f.edge({41,11},{50,11},1031,1031,1031,1));
  route_732.push_back(f.edge({50,11},{54,11},1031,1030,1027,1));
  route_732.push_back(f.edge({54,11},{54,10},1027,244,248,1));
  f.terminal(route_732,{54,10},248,86,-1,-1); route_732.pop_back();
  route_732.push_back(f.pin({54,10},86,false));
  auto &installed_732=install(732,std::move(route_732));
  f.edge({49,10},{54,14},160,1900,1900);
  for(int n=0;n<512;++n) {
    f.edge({54,14},{54,13},1900,2000+n,2000+n);
    f.arc({54,13,fpga::CB_NODE_DST,2000+n},{54,13,fpga::CB_NODE_LOCAL,3000+n});
  }
  f.rebuild(); f.device.rebuildIncomingDstMasks();
  auto &signal=f.signal(730);
  auto history=[&] {
    std::ostringstream out;
    fpga::dumpNetRouteHistory(f.signal(731).net,out);
    fpga::dumpNetRouteHistory(f.signal(732).net,out);
    return out.str();
  };
  const auto before=f.state(), other_before=history();
  std::ostringstream witness;
  const auto &binding=signal.net.routes[0];
  require(fpga::auditRouteCompletion(signal.net,binding,installed_730.front(),
      installed_730.back(),witness,{},4096,4096,200) &&
      witness.str().find("SOURCE_DIAGNOSTIC_COMPLETE nodes=65 cuts=0")!=std::string::npos,
      "captured free 65-node bypass is not a complete legal route");
  require(f.state()==before,"read-only witness changed the three-signal snapshot");
  pnr::RouteDesign router; router.fpga=&f.device;
  router.moving_stage=true; router.route_preemption_enabled=true;
  router.moving_focus_inst=&signal.sinks[0];
  for(auto &[id,s]:f.signals) {s->driver.outline.fixed=true; for(auto &sink:s->sinks) sink.outline.fixed=true;}
  pnr::RouteDesign::RouteTask task{.from=&signal.driver,.to=&signal.sinks[0],.net=&signal.net,
      .from_port="out",.to_port="in",.net_name=binding.route_name};
  const bool done=router.repairTaskGaps(task,0,256,true);
  std::cout << "Fixed Moving LOCAL probe: complete=" << done
      << " cuts=" << router.route_stats.preempt_success << '\n';
  require(done,"Moving did not find its free resolved-LOCAL bypass");
  require(fpga::isRouteComplete(installed_730) && fpga::isRouteComplete(installed_731) &&
      fpga::isRouteComplete(installed_732),"Moving repeated the three-route preemption cycle");
  require(router.route_stats.preempt_success==0 && router.pending_route_todo.empty() &&
      history()==other_before,"a free Moving repair displaced a completed competitor");
  f.state();
  std::cout << "Fixed Moving LOCAL cycle: all three complete; no cuts; budget=256\n";
}

void gap_source_frontier_reuse_regression() {
  bool all_reused = true;
  for (int length : {72, 96}) for (bool allow_cut : {false, true}) {
    Fixture f(5, length + 2);
    auto first = f.edge({1,1},{2,1},144,200,500,0);
    first.owns_landing = true;
    Wire gap;
    gap.type = Wire::WIRE_GAP;
    gap.from = {2,1}; gap.from_node_type = fpga::CB_NODE_DST; gap.from_node = 500;
    gap.to = {length,1}; gap.to_node_type = fpga::CB_NODE_LOCAL; gap.to_node = 91;
    auto &route = f.install(718, {f.pin({1,1},144,true), first, gap,
        f.pin({length,1},91,false)});
    f.tile({1,1}).pin_state.leased_nodes.clearBit(144);
    for (int x = 2; x < length; ++x) f.edge({x,1},{x+1,1},500,200,500);
    f.arc({length,1,fpga::CB_NODE_DST,500},{length,1,fpga::CB_NODE_LOCAL,91});
    Route victim{f.pin({length,3},3990,true),
        f.edge({length,3},{length,2},3990,3990,3990,0),
        f.edge({length,2},{length,1},3990,3990,500),
        f.edge({length,1},{length,0},500,3990,3990)};
    f.terminal(victim,{length,0},3990,3990);
    f.install(719,std::move(victim));
    f.tile({length,3}).pin_state.leased_nodes.clearBit(3990);
    f.rebuild(); f.device.rebuildIncomingDstMasks();
    const auto before = f.state();
    int commits = 0;
    auto preempt = [&](const std::vector<fpga::RouteCutNode> &cuts, bool commit,
                        std::vector<fpga::RouteCutNode> &released) {
      if (!allow_cut || cuts.empty()) return false;
      for (const auto &cut : cuts)
        if (!same(cut.tile,{length,1}) || cut.type != fpga::CB_NODE_DST || cut.node != 500)
          return false;
      const auto victims = fpga::findNetRouteCutVictims(cuts);
      if (victims.empty()) return false;
      released = fpga::previewNetRouteCuts(victims,cuts,true);
      if (!commit) return !released.empty();
      ++commits;
      return fpga::unrouteNetRoutesFromNodes(victims,cuts,true);
    };
    auto &net = f.signal(718).net;
    require(!fpga::repairNetRouteGaps(net,0,[]{return true;},4096,200,{},preempt) &&
        f.state() == before, "cancelled GAP probe changed routing state");
    SourceRepairTrace trace(net.name);
    const bool done = fpga::repairNetRouteGaps(net,0,{},4096,200,{},preempt);
    require(done == allow_cut && commits == int(allow_cut),
        "GAP frontier reuse changed the cut policy or committed a preview");
    if (allow_cut) {
      require(fpga::isRouteComplete(route), "GAP reuse left an incomplete route");
      check_complete_path(f,route,length-1);
    } else require(f.state() == before, "refused GAP cut changed leases or fragments");
    f.state(); // Audit every installed lease against its real route owners.
    std::ifstream log(trace.path);
    size_t searches = 0, reused = 0;
    for (std::string line; std::getline(log,line);) {
      if (!line.starts_with("SOURCE_SEARCH end ")) continue;
      ++searches;
      if (line.find("reused_frontier=1") != std::string::npos) {
        ++reused;
        require(line.find("expanded=0 ") != std::string::npos,
            "GAP fallback expanded the unchanged free graph again");
      }
    }
    all_reused &= searches == 2 && reused == 1;
    std::cout << "GAP source frontier length=" << length << " cut=" << allow_cut
        << " source_searches=" << searches << " reused=" << reused << '\n';
  }
  require(all_reused, "GAP fallback repeated its early free source search");
}

// Last Example 1 timeout: preserve the witnessed seventeen numeric nodes,
// seven jumps, internal LOCAL bridge and the final SRC/DST transit cut pair.
// Unrelated retained fragments are reduced to a trapped GAP and a broad free
// exit fan. Moving's preliminary source probe must not flood that old GAP
// before trying the already-reached Grounding approach. Work is bounded by
// cancellation polls, not machine speed. Every busy bit has a real owner;
// all textual resource and cell names are generated by the abstract fixture.
void moving_source_probe_regression() {
  for (bool allow_cut : {false,true}) {
    Fixture f(24,70);
    auto first=f.edge({58,14},{10,20},159,700,700,0);
    first.owns_landing=true;
    Wire gap; gap.type=Wire::WIRE_GAP; gap.from={10,20}; gap.to={58,6};
    gap.from_node_type=gap.to_node_type=fpga::CB_NODE_DST;
    gap.from_node=700; gap.to_node=300;
    Route retained{f.pin({58,14},159,true),first,gap};
    f.terminal(retained,{58,6},300,96);
    auto &route=f.install(720,std::move(retained));
    f.tile({58,14}).pin_state.leased_nodes.clearBit(159);
    for(int n=1000;n<3000;++n) {
      f.edge({10,20},{10,19},700,n,n);
      // A landing needs a physical continuation to belong to valid_dst_nodes.
      // These terminal leaves make real exact-gap work, without a sink path.
      f.arc({10,19,fpga::CB_NODE_DST,n},{10,19,fpga::CB_NODE_LOCAL,n});
    }
    f.edge({58,14},{63,10},159,976,976,0);
    f.edge({63,10},{66,11},976,544,544);
    f.edge({66,11},{66,10},544,241,245);
    f.edge({66,10},{63,9},245,3808,3808);
    f.edge({63,9},{63,3},3808,160,160);
    f.arc({63,3,fpga::CB_NODE_DST,160},{63,3,fpga::CB_NODE_LOCAL,165});
    f.edge({63,3},{58,7},165,3376,3376,0);
    f.edge({58,7},{58,6},3376,247,241);
    f.arc({58,6,fpga::CB_NODE_DST,241},{58,6,fpga::CB_NODE_LOCAL,96});
    Route foreign{f.pin({58,9},3990,true),
        f.edge({58,9},{58,7},3990,3990,3990,0),
        f.edge({58,7},{58,6},3990,247,241),
        f.edge({58,6},{58,5},241,3990,3990)};
    f.terminal(foreign,{58,5},3990,3990);
    f.install(721,std::move(foreign));
    f.tile({58,9}).pin_state.leased_nodes.clearBit(3990);
    f.rebuild(); f.device.rebuildIncomingDstMasks();
    for(int n=1000;n<3000;++n) {
      const auto targets=f.device.resolveJumpTargets(f.tile({10,20}),n);
      require(targets.size()==1 && targets.front().dst_node==n,
          "source-probe background exit has no physical landing");
    }
    const auto before=f.state();
    auto &net=f.signal(720).net;
    int commits=0;
    auto preempt=[&](const std::vector<fpga::RouteCutNode> &cuts,bool commit,
                      std::vector<fpga::RouteCutNode> &released) {
      if(!allow_cut || cuts.empty()) return false;
      for(const auto &cut:cuts)
        if(!((same(cut.tile,{58,7}) && cut.type==fpga::CB_NODE_SRC && cut.node==247) ||
             (same(cut.tile,{58,6}) && cut.type==fpga::CB_NODE_DST && cut.node==241))) return false;
      const auto victims=fpga::findNetRouteCutVictims(cuts);
      if(victims.empty()) return false;
      released=fpga::previewNetRouteCuts(victims,cuts,true);
      if(!commit) return !released.empty();
      ++commits;
      return fpga::unrouteNetRoutesFromNodes(victims,cuts,true);
    };
    require(!fpga::completeNetRouteFromSource(net,0,[]{return true;},512,200,preempt) &&
        f.state()==before,"cancelled source probe changed a retained owner");
    require(!fpga::completeNetRouteFromSource(net,0,{},1,200,preempt) && f.state()==before,
        "source probe bypassed its state limit");
    require(!fpga::completeNetRouteFromSource(net,0,{},512,6,preempt) && f.state()==before,
        "source probe bypassed its hop limit");
    auto &bridge=*f.tile({63,3}).cb_type;
    bridge.dst_local[160].local.clearBit(165); f.rebuild();
    require(!fpga::completeNetRouteFromSource(net,0,{},512,200,preempt) && f.state()==before,
        "source probe invented its required internal LOCAL bridge");
    bridge.dst_local[160].local.setBit(165); f.rebuild();
    unsigned polls=0;
    const bool done=fpga::completeNetRouteFromSource(net,0,[&]{return ++polls>1400;},512,200,preempt);
    std::cout << "Moving source probe cut=" << allow_cut << " done=" << done
        << " cancellation_polls=" << polls << '\n';
    require(done==allow_cut,"preliminary Moving source probe flooded the unrelated retained GAP");
    require(commits==int(allow_cut),"source probe committed a preview or missed the combined cut");
    if(allow_cut) {
      check_complete_path(f,route,7);
      require(fpga::isRouteComplete(route) && f.tile({63,3}).cb.local.local.testBit(165),
          "source probe lost its seven-hop path or internal LOCAL bridge");
    } else require(f.state()==before,"refused source probe changed its owners");
    f.state();
  }
}

// Example 1 run 4 ended with an EMPTY active queue and only fixed endpoints
// in deferred work. The relocation-only admission gate rejected them before
// the existing wire-only handler ran. Preserve the exact eighteen-node free
// witness (seven jumps and two terminal JOINTs), with abstract names. Unlike
// the older fixed-endpoint tests, start with no active routing task or binding.
void moving_deferred_fixed_wire_regression() {
  for(bool io : {false,true}) {
    Fixture f(30,44);
    const Coord from{40,8}, to{36,27};
    f.edge(from,{36,12},147,3378,3378,0);
    f.edge({36,12},{36,14},3378,66,66);
    f.edge({36,14},{32,14},66,3845,3841);
    f.edge({32,14},{31,15},3841,3617,3617);
    f.edge({31,15},{31,21},3617,97,97);
    f.edge({31,21},{36,25},97,817,817);
    f.edge({36,25},to,817,17,20);
    Route terminal;
    f.terminal(terminal,to,20,87,5,19);
    auto &s=f.signal(723);
    s.driver.tile.set(&f.device.tile_grid[from.y*f.device.size_width+from.x]);
    s.driver.coord=from; s.driver.outline.fixed=true;
    s.sinks.emplace_back(); s.sink_cells.emplace_back();
    auto &sink=s.sinks.back();
    s.sink_cells.back().name="receiver_723";
    s.sink_cells.back().type=io?"IOBUF":"logic_731";
    sink.cell_ref.set(&s.sink_cells.back());
    sink.tile.set(&f.device.tile_grid[to.y*f.device.size_width+to.x]);
    sink.coord=to; sink.outline.fixed=!io;
    f.rebuild(); f.device.rebuildIncomingDstMasks();
    const Wire source=f.pin(from,147,true), input=f.pin(to,87,false);
    rtl::NetRouteBinding binding;
    binding.from=&s.driver; binding.to=&sink; binding.from_port="out";
    binding.to_port="in"; binding.route_name="deferred_723";
    const auto before=f.state();
    std::ostringstream witness;
    require(fpga::auditRouteCompletion(s.net,binding,source,input,witness,{},32768,32768,200) &&
        witness.str().find("SOURCE_DIAGNOSTIC_COMPLETE nodes=18 cuts=0")!=std::string::npos,
        "deferred fixed fixture lost the captured eighteen-node free approach");
    require(sink.wires.empty() && f.state()==before,"read-only deferred witness changed occupancy");
    require(!pnr::movingDeferredTaskEligible(true,false,true,true,false) &&
        !pnr::movingDeferredTaskEligible(false,false,false,true,false) &&
        !pnr::movingDeferredTaskEligible(false,false,true,false,false),
        "deferred admission bypassed source, placement or cooldown restrictions");
    pnr::RouteDesign::RouteTask task{.from=&s.driver,.to=&sink,.net=&s.net,
        .from_port="out",.to_port="in",.net_name=binding.route_name};
    std::vector<pnr::RouteDesign::RouteTask> active, deferred{task};
    require(active.empty() && deferred.size()==1 && s.net.routes.empty(),
        "deferred regression unexpectedly started with active or installed work");
    const auto &candidate=deferred.back();
    const bool movable=!candidate.to->outline.fixed && !io;
    if(pnr::movingDeferredTaskEligible(false,movable,candidate.to->tile.peer!=nullptr,
                                     candidate.to->outline.fixed,io)) {
      active.push_back(std::move(deferred.back())); deferred.pop_back();
    }
    require(active.size()==1 && deferred.empty(),
        "no movable endpoint hid a deferred fixed/IO task with a proven free wire path");
    require(fpga::routeUnboundNetFromSource(s.net,binding,source,input,{},32768,200) &&
        sink.wires.size()==1 && fpga::isRouteComplete(sink.wires.front()),
        "selected deferred fixed task did not finish its free source-to-input route");
    check_complete_path(f,sink.wires.front(),7);
    require(same(s.driver.coord,from) && same(sink.coord,to) && s.driver.outline.fixed &&
        sink.outline.fixed==!io,"deferred wire recovery changed fixed/IO placement");
    f.state();
    std::cout << "deferred fixed wire io=" << io << " seven_hops=1 active=0->1 complete=1\n";
  }
}

// Example 1 run 5 stopped after the THIRD successful reverse-boundary release:
// its caller allowed only two retries and never tried the now-changed graph.
// Model three separately owned transit obstructions, with their real Takeoff
// prefixes and terminal tails. The fourth reverse search can reach the sole
// legal source, inside the unchanged ten-hop limit. Every cut is audited;
// afterwards all displaced owners must repair through pre-existing bypasses.
void moving_source_boundary_continuation_regression() {
  for(bool allow_cut : {false,true}) {
    Fixture f(4,7);
    for(int x=1;x<=4;++x)
      f.edge({x,1},{x+1,1},x==1?147:198+x,99+x,199+x,x==1?0:1);
    Route endpoint;
    f.terminal(endpoint,{5,1},203,87);
    for(int n=0;n<3;++n) {
      const int x=n+1;
      Route owner{f.pin({x,0},3990,true),
          f.edge({x,0},{x,1},3990,900,300+n,0),
          f.edge({x,1},{x+1,1},300+n,100+n,200+n),
          f.edge({x+1,1},{x+1,2},200+n,950,350)};
      f.terminal(owner,{x+1,2},350,3990);
      f.install(731+n,std::move(owner));
      f.tile({x,0}).pin_state.leased_nodes.clearBit(3990);
      f.edge({x,1},{x+1,2},300+n,1100+n,351);
      Route bypass;
      f.terminal(bypass,{x+1,2},351,3990);
    }
    f.rebuild(); f.device.rebuildIncomingDstMasks();
    const auto before=f.state();
    size_t releases=0, searches=0;
    std::set<std::tuple<int,int,int>> released_nodes;
    auto release=[&](const std::vector<pnr::DockingBridgeBlocker> &blocked) {
      if(!allow_cut) return false;
      for(const auto &edge:blocked) {
        if(!edge.dst_busy || edge.dst<200 || edge.dst>202) continue;
        const std::vector<fpga::RouteCutNode> cuts{{edge.tile,fpga::CB_NODE_DST,edge.dst}};
        const auto victims=fpga::findNetRouteCutVictims(cuts);
        if(victims.empty() || fpga::previewNetRouteCuts(victims,cuts,true).empty()) continue;
        auto &tile=f.tile(edge.tile);
        require(tile.cb.dst.jump.testBit(edge.dst),"source boundary was not actually occupied");
        require(released_nodes.emplace(edge.tile.x,edge.tile.y,edge.dst).second,
            "source retry released the same boundary twice");
        require(fpga::unrouteNetRoutesFromNodes(victims,cuts,true),"source transit cut failed");
        require(!tile.cb.dst.jump.testBit(edge.dst) &&
            fpga::findNetOwnersByNode(tile,fpga::CB_NODE_DST,edge.dst).empty(),
            "source boundary release left a lease or owner behind");
        ++releases;
        f.state();
        return true;
      }
      return false;
    };
    auto placement=[&](Tile &tile,int src,pnr::BackwardTakeoffChoice &choice) {
      if(!same(tile.coord,{1,1}) || src!=100) {
        choice.counts_toward_probe_limit=false;
        return false;
      }
      choice={147,-1,-1};
      return true;
    };
    auto search=[&](const std::function<bool()> &cancel) {
      return pnr::routeBackwardToTakeoff(f.tile({5,1}),NodeMask{0,1}<<87,{1,1},
          pnr::movingSourceSearchDepth(),6,-1,placement,nullptr,cancel,32768,0,0,
          nullptr,nullptr,{}, {},release);
    };
    const auto cancelled=search([]{return true;});
    require(!cancelled.success && !cancelled.boundary_released && !releases && f.state()==before,
        "cancelled source continuation cut an owner");
    pnr::BackwardTakeoffRoute result;
    do {
      ++searches;
      result=search({});
      if(result.success || !pnr::movingSourceRetriesReleasedBoundary(result.boundary_released)) break;
    } while(true);
    std::cout << "source boundary continuation cut=" << allow_cut << " searches=" << searches
        << " releases=" << releases << " complete=" << result.success << '\n';
    if(!allow_cut) {
      require(!result.success && !releases && searches==1 && f.state()==before,
          "refused source boundary changed routing or retried without progress");
      continue;
    }
    require(result.success && searches==4 && releases==3 && result.source_tile==&f.tile({1,1}),
        "source caller stopped after two topology revisions before trying the complete path");
    Route route{f.pin({1,1},147,true)};
    route.insert(route.end(),result.fragments.begin(),result.fragments.end());
    auto &installed=f.install(730,std::move(route));
    f.tile({1,1}).pin_state.leased_nodes.clearBit(147);
    require(fpga::isRouteComplete(installed),"source continuation committed an incomplete trunk");
    check_complete_path(f,installed,4);
    f.state();
    for(int n=0;n<3;++n) {
      auto &victim=f.signal(731+n);
      require(fpga::repairNetRouteGaps(victim.net,0) &&
          fpga::isRouteComplete(victim.sinks.front().wires.front()),
          "displaced transit owner could not use its generated free bypass");
      check_complete_path(f,victim.sinks.front().wires.front(),2);
    }
    f.state();
  }
}

// A 50K failure exhausted the fast 65536-label cap on EVERY retry even when
// the caller allowed 524288 labels. A read-only probe found a free 96-node
// completion at 411120 labels, with clean ownership at both endpoints.
// Scale that mismatch down: a shared trunk, a free corridor longer than the
// fast cap, and an independently owned alternate destination entry.
void source_free_budget_escalation_regression() {
  const char *key="SCALEPNR_FREE_SEARCH_STATES";
  const bool present=std::getenv(key)!=nullptr;
  const std::string original=present?std::getenv(key):"";
  struct Restore {
    const char *key; bool present; std::string value;
    ~Restore() { if(present) setenv(key,value.c_str(),1); else unsetenv(key); }
  } restore{key,present,original};
  for(int length:{8,24,48}) for(int cap:{4,16}) for(bool offer_preemption:{false,true}) {
    setenv(key,std::to_string(cap).c_str(),1);
    Fixture f(6,length+2);
    Route seed{f.pin({1,2},144,true),f.edge({1,2},{2,2},144,200,500,0),
        f.edge({2,2},{2,3},500,201,501)};
    f.terminal(seed,{2,3},501,90);
    f.install(715,std::move(seed));
    f.tile({1,2}).pin_state.leased_nodes.clearBit(144);
    for(int x=2;x<length;++x) f.edge({x,2},{x+1,2},500,200,500);
    // The recorded free witness crosses an internal LOCAL and three JOINTs.
    const int middle=length/2;
    f.tile({middle,2}).cb_type->dst_src[500].jump.clearBit(200);
    f.arc({middle,2,fpga::CB_NODE_DST,500},{middle,2,fpga::CB_NODE_LOCAL,180});
    f.arc({middle,2,fpga::CB_NODE_LOCAL,180},{middle,2,fpga::CB_NODE_SRC,200});
    f.arc({length,2,fpga::CB_NODE_DST,500},{length,2,fpga::CB_NODE_JOINT,54});
    f.arc({length,2,fpga::CB_NODE_JOINT,54},{length,2,fpga::CB_NODE_JOINT,39});
    f.arc({length,2,fpga::CB_NODE_JOINT,39},{length,2,fpga::CB_NODE_JOINT,19});
    f.arc({length,2,fpga::CB_NODE_JOINT,19},{length,2,fpga::CB_NODE_LOCAL,91});
    Route victim{f.pin({length,4},3990,true),
        f.edge({length,4},{length,2},3990,3990,501,0),
        f.edge({length,2},{length,1},501,3990,3990)};
    f.terminal(victim,{length,1},3990,3990);
    f.install(716,std::move(victim));
    f.tile({length,4}).pin_state.leased_nodes.clearBit(3990);
    f.arc({length,2,fpga::CB_NODE_DST,501},{length,2,fpga::CB_NODE_JOINT,19});
    auto &s=f.signal(715);
    s.sinks.emplace_back(); s.sink_cells.emplace_back();
    auto &sink=s.sinks.back();
    s.sink_cells.back().name="receiver_715"; s.sink_cells.back().type="logic_731";
    sink.cell_ref.set(&s.sink_cells.back());
    sink.tile.set(&f.device.tile_grid[2*f.device.size_width+length]);
    rtl::NetRouteBinding binding;
    binding.from=&s.driver; binding.to=&sink;
    binding.from_port="out"; binding.to_port="in"; binding.route_name="branch_715_extra";
    const auto from=f.pin({1,2},144,true), to=f.pin({length,2},91,false);
    f.rebuild(); f.device.rebuildIncomingDstMasks();
    const auto before=f.state();
    // The caller's actual bound is never exceeded, and failure leases nothing.
    require(!fpga::routeUnboundNetFromSource(s.net,binding,from,to,{},cap,100) &&
        f.state()==before && sink.wires.empty(),"small caller budget changed routing");
    // Cancellation must also stop the last-chance free retry without mutation.
    require(!fpga::routeUnboundNetFromSource(s.net,binding,from,to,[]{return true;},4096,100) &&
        f.state()==before,"cancelled escalation changed routing");
    auto &terminal=f.tile({length,2}).cb_type->joint_local[19].local;
    terminal.clearBit(91); f.rebuild();
    // More budget does not invent a missing endpoint or borrow a foreign route.
    require(!fpga::routeUnboundNetFromSource(s.net,binding,from,to,{},4096,100) &&
        f.state()==before,"disconnected escalation changed routing");
    terminal.setBit(91); f.rebuild();
    std::ostringstream owner_before;
    fpga::dumpNetRouteHistory(f.signal(716).net,owner_before);
    RepairDecisionTrace trace;
    fpga::RepairGroundingPreemption preempt;
    if(offer_preemption) preempt=[](const auto &,bool commit,auto &) {
      // A declined preview never authorizes changing this real transit owner.
      require(!commit,"free-only escalation tried to commit a transit cut");
      return false;
    };
    const bool done=fpga::routeUnboundNetFromSource(s.net,binding,from,to,{},4096,100,preempt);
    std::printf("free-budget escalation length=%d cap=%d preemption=%d complete=%d\n",
        length,cap,offer_preemption,done);
    // The larger explicit request must not silently repeat the small cap forever.
    require(done,"BUG: fast free-search cap hides a free path within the caller's larger budget");
    check_complete_path(f,sink.wires.front(),length-1);
    require(fpga::isRouteComplete(s.sinks.front().wires.front()),"escalation damaged the trunk");
    std::ostringstream owner_after;
    fpga::dumpNetRouteHistory(f.signal(716).net,owner_after);
    require(owner_before.str()==owner_after.str(),"free escalation changed the transit owner");
    f.state(); // Every installed busy node must still have its real route owner.
    const auto decisions=trace.read();
    require(decisions.find("limit="+std::to_string(cap)+" reached=0")!=std::string::npos,
        "regression did not exercise the initial capped failure");
    // Existing completion searches can already solve a short free corridor
    // when offered a cut callback. Preserve that success; without the callback
    // only the new final free-only retry can honor the caller's larger budget.
    if(!offer_preemption)
      require(decisions.find("limit=4096 reached=1")!=std::string::npos,
          "free-only regression did not exercise full-budget completion");
    require(fpga::unrouteNet(s.net),"cannot tear down the repaired signal");
    f.state(); // Shared trunk/branch cleanup must not leak or clear foreign leases.
  }
}

// A larger free frontier can reach Grounding without reaching the occupied
// final DST. The 50K replay proved this with a clean owner audit and a single
// legal transit cut. A free-only escalation loses that completion opportunity.
void source_grounding_budget_escalation_regression() {
  const char *key="SCALEPNR_FREE_SEARCH_STATES";
  const bool present=std::getenv(key)!=nullptr;
  const std::string original=present?std::getenv(key):"";
  struct Restore {
    const char *key; bool present; std::string value;
    ~Restore() { if(present) setenv(key,value.c_str(),1); else unsetenv(key); }
  } restore{key,present,original};
  for(int cap:{4,8}) for(size_t budget:{64,128}) for(bool allow_cut:{false,true}) {
    setenv(key,std::to_string(cap).c_str(),1);
    Fixture f(16,44);
    Route seed{f.pin({1,2},144,true),f.edge({1,2},{2,2},144,200,500,0),
        f.edge({2,2},{2,3},500,201,501)};
    f.terminal(seed,{2,3},501,90);
    f.install(717,std::move(seed));
    f.tile({1,2}).pin_state.leased_nodes.clearBit(144);
    // The productive free detour initially moves away, then jumps close to
    // the sink. The fast cap cannot reach it; the explicit budget can.
    f.edge({2,2},{2,12},500,250,500);
    f.edge({2,12},{40,4},500,250,500);
    f.edge({40,4},{40,2},500,250,500);
    f.arc({40,2,fpga::CB_NODE_DST,500},{40,2,fpga::CB_NODE_JOINT,24});
    f.arc({40,2,fpga::CB_NODE_JOINT,24},{40,2,fpga::CB_NODE_LOCAL,91});
    // Nearby dead corridors exhaust the distinct Takeoff search's budget.
    // This makes the regression exercise expanded-free-frontier Grounding,
    // rather than accidentally succeeding through that earlier fallback.
    f.edge({2,2},{3,2},500,200,500);
    f.arc({3,2,fpga::CB_NODE_DST,500},{3,2,fpga::CB_NODE_JOINT,0});
    for(int j=0;j<12;++j)
      f.arc({3,2,fpga::CB_NODE_JOINT,j},{3,2,fpga::CB_NODE_JOINT,j+1});
    for(int j=20;j<532;++j) {
      f.arc({3,2,fpga::CB_NODE_JOINT,12},{3,2,fpga::CB_NODE_JOINT,j});
      f.arc({3,2,fpga::CB_NODE_JOINT,j},{3,2,fpga::CB_NODE_SRC,300});
    }
    f.arc({3,2,fpga::CB_NODE_SRC,300},{4,2,fpga::CB_NODE_DST,502});
    Route victim{f.pin({40,8},3990,true),f.edge({40,8},{40,6},3990,3990,3990,0),
        f.edge({40,6},{40,2},3990,3990,500),
        f.edge({40,2},{40,1},500,3990,3990)};
    f.terminal(victim,{40,1},3990,3990);
    f.install(718,std::move(victim));
    f.tile({40,8}).pin_state.leased_nodes.clearBit(3990);
    auto &s=f.signal(717);
    s.sinks.emplace_back(); s.sink_cells.emplace_back();
    auto &sink=s.sinks.back();
    s.sink_cells.back().name="receiver_717"; s.sink_cells.back().type="logic_731";
    sink.cell_ref.set(&s.sink_cells.back());
    sink.tile.set(&f.device.tile_grid[2*f.device.size_width+40]);
    rtl::NetRouteBinding binding;
    binding.from=&s.driver; binding.to=&sink;
    binding.from_port="out"; binding.to_port="in"; binding.route_name="branch_717_extra";
    const auto from=f.pin({1,2},144,true), to=f.pin({40,2},91,false);
    f.rebuild(); f.device.rebuildIncomingDstMasks();
    const auto before=f.state();
    int commits=0;
    auto preempt=[&](const std::vector<fpga::RouteCutNode> &cuts,bool commit,
                      std::vector<fpga::RouteCutNode> &released) {
      if(!allow_cut || cuts.size()!=1 || !same(cuts.front().tile,{40,2}) ||
          cuts.front().type!=fpga::CB_NODE_DST || cuts.front().node!=500) return false;
      const auto victims=fpga::findNetRouteCutVictims(cuts);
      // Only the real transit owner at the final input is an eligible victim.
      if(victims.empty()) return false;
      for(const auto &v:victims)
        require(v.net==&f.signal(718).net,"grounding selected a different signal");
      released=fpga::previewNetRouteCuts(victims,cuts,true);
      if(!commit) return !released.empty();
      ++commits;
      return fpga::unrouteNetRoutesFromNodes(victims,cuts,true);
    };
    // Cancellation and the small explicit bound must leave every owner intact.
    require(!fpga::routeUnboundNetFromSource(s.net,binding,from,to,[]{return true;},budget,100,preempt)
        && f.state()==before && !commits,"cancelled grounding escalation mutated routing");
    require(!fpga::routeUnboundNetFromSource(s.net,binding,from,to,{},cap,100,preempt)
        && f.state()==before && !commits,"small-budget grounding escalation mutated routing");
    RepairDecisionTrace trace;
    const bool done=fpga::routeUnboundNetFromSource(s.net,binding,from,to,{},budget,100,preempt);
    std::cout << "grounding-budget escalation cap=" << cap << " budget=" << budget
        << " allow_cut=" << allow_cut << " complete=" << done << '\n' << trace.read();
    // Reaching a free prefix is useful even when the last DST requires a cut.
    require(done==allow_cut,"BUG: expanded free frontier discarded its legal grounding completion");
    require(commits==int(allow_cut),"grounding escalation committed the wrong number of cuts");
    if(done) {
      check_complete_path(f,sink.wires.front(),4);
      // The shared original trunk stays complete; every remaining bit has an owner.
      require(fpga::isRouteComplete(s.sinks.front().wires.front()),"grounding damaged original trunk");
      f.state();
      require(fpga::unrouteNet(s.net),"cannot remove grounded signal");
      f.state(); // Cleanup cannot leave unowned leases or erase the transit prefix.
    } else {
      require(sink.wires.empty() && f.state()==before,"declined grounding changed leases");
    }
  }
}

// Chunk 18's unbound source search exhausted 131072 states despite a free
// completion. Replay its two recorded free frontiers and diagnostic witness;
// names are generated, occupied alternatives have real independent owners.
// The deadline capture is a positive control, not a reproduced timeout:
// its free graph must route under existing production budgets without cuts.
void unbound_completion_frontier_regression(bool deadline_capture) {
  Fixture f(208,144);
  f.device.cb_types.reserve(4096);
  f.loadFrontier(deadline_capture ? "unbound_deadline_frontier.txt" :
      "unbound_completion_frontier.txt",deadline_capture ? 81225 : 750195);
  const Wire from=deadline_capture ? f.pin({82,115},146,true) : f.pin({41,184},144,true);
  const Wire to=deadline_capture ? f.pin({71,48},81,false) : f.pin({53,98},91,false);
  auto &s=f.signal(713);
  s.driver.tile.set(&f.device.tile_grid[from.from.y*144+from.from.x]);
  s.sinks.emplace_back();
  s.sink_cells.emplace_back();
  auto &sink=s.sinks.back();
  s.sink_cells.back().name="receiver_713";
  s.sink_cells.back().type="logic_731";
  sink.cell_ref.set(&s.sink_cells.back());
  sink.tile.set(&f.device.tile_grid[to.from.y*144+to.from.x]);
  rtl::NetRouteBinding endpoints;
  endpoints.from=&s.driver; endpoints.to=&sink;
  endpoints.from_port="out"; endpoints.to_port="in"; endpoints.route_name="branch_713";

  // A busy destination alternative must stay leased to its original route.
  const Coord owner_source{to.from.x,to.from.y+1};
  Route occupied{f.pin(owner_source,3990,true),
      f.edge(owner_source,to.from,3990,3990,3811,0)};
  f.terminal(occupied,to.from,3811,3990);
  f.install(714,std::move(occupied));
  auto owner_state=[&] {
    std::ostringstream out;
    fpga::dumpNetRouteHistory(f.signal(714).net,out);
    return out.str();
  };
  const auto original=owner_state();
  f.tile(owner_source).pin_state.leased_nodes.clearBit(3990);
  f.rebuild(); f.device.rebuildIncomingDstMasks();
  const auto before=f.state();

  // Cancellation cannot manufacture a binding or leave a partial lease.
  require(!fpga::routeUnboundNetFromSource(s.net,endpoints,from,to,[]{return true;},
      fpga::UNBOUND_ROUTE_STATE_BUDGETS.back()),"cancelled completion succeeded");
  require(f.state()==before && sink.wires.empty(),"cancelled completion mutated state");
  bool done=false;
  RepairDecisionTrace trace;
  for(size_t budget:fpga::UNBOUND_ROUTE_STATE_BUDGETS) {
    done=fpga::routeUnboundNetFromSource(s.net,endpoints,from,to,{},budget);
    if(done) break;
    // Every failed attempt is read-only, including registrations and masks.
    require(f.state()==before && sink.wires.empty(),"bounded failure leaked a partial route");
  }
  std::cout << trace.read();
  require(done,"production unbound budgets discarded the recorded free completion");
  // The committed route must reach the requested pin, with no gaps or cuts.
  require(s.net.routes.size()==1 && sink.wires.size()==1 &&
      fpga::isRouteComplete(sink.wires.front()),"completion did not create one full binding");
  const auto &route=sink.wires.front();
  require(same(route.front().from,from.from) && route.front().local==from.local &&
      same(route.back().from,to.from) && route.back().local==to.local,"completion moved endpoints");
  require(owner_state()==original,
      "free completion changed the foreign destination owner");
  f.state();
  // Teardown must release only the requester; the foreign route survives.
  require(fpga::unrouteNetRoute(s.net,0),"cannot unroute completed request");
  f.state();
  require(f.tile(to.from).cb.dst.jump.testBit(3811),"unroute released a foreign destination");
  require(fpga::unrouteNetRoute(f.signal(714).net,0),"cannot remove fixture owner");
  f.state();
}

// Scaling regression for the next Example 1 stop (Fanouts deadline). Samples
// repeatedly found exact-gap repair scanning every Tile-registered route's
// fragments for busy SRCs that have no reusable same-source incoming arc.
// Keep its (50,34)/DST256 -> (54,42)/DST544 gap, but reduce the unrelated
// fabric to 32 real transit owners crossing 16 search Tiles. No naked busy
// bits or vendor node names. Assert expensive-query counts, not wall time.
// The second case requires genuine same-source reuse and its owner checks.
void fanout_gap_owner_scan_regression(bool source_only) {
  for(bool shared:{false,true}) {
    if(source_only && shared) continue;
    Fixture f(80,80);
    Route r{f.pin({49,34},154,true),f.edge({49,34},{50,34},154,256,256,0)};
    r.back().owns_landing=true;
    Wire gap; gap.type=Wire::WIRE_GAP; gap.from={50,34}; gap.to={54,42};
    gap.from_node_type=gap.to_node_type=fpga::CB_NODE_DST;
    gap.from_node=256; gap.to_node=544; r.push_back(gap);
    f.terminal(r,{54,42},544,110);
    auto &route=f.install(900001,std::move(r));
    auto &s=f.signal(900001);
    Route sibling{f.pin({49,34},154,true),f.edge({49,34},{50,34},154,256,256,0)};
    for(int x=50;x<65;++x) {
      // A LOCAL routing bridge forces the second test into the numeric
      // whole-source fallback, which had the same expensive negative lookup.
      if(source_only && x==50) {
        f.tile({50,34}).cb_type->dst_local[256].local.setBit(3901);
        sibling.push_back(f.edge({50,34},{51,34},3901,3000,3001,0));
      } else sibling.push_back(f.edge({x,34},{x+1,34},x==50?256:3001,3000,3001));
    }
    f.edge({65,34},{54,42},3001,3000,544);
    if(shared) {
      sibling.push_back(f.edge({65,34},{66,34},3001,3002,3002));
      f.terminal(sibling,{66,34},3002,3900);
      f.install(900001,std::move(sibling));
    }
    for(int n=0;n<32;++n) {
      const int node=1000+n;
      Route owner{f.pin({49,33},100+n,true),
          f.edge({49,33},{50,34},100+n,node,node,0)};
      for(int x=50;x<=65;++x) {
        owner.push_back(f.edge({x,34},{x+1,34},node,node,node));
        f.tile({x,34}).cb_type->dst_src[x==50?256:3001].jump.setBit(node);
      }
      f.terminal(owner,{66,34},node,200+n);
      f.install(910000+n,std::move(owner));
    }
    f.rebuild(); f.state();
    std::ostringstream owners_before;
    for(int n=0;n<32;++n) fpga::dumpNetRouteHistory(f.signal(910000+n).net,owners_before);
    std::string decisions;
    {
      RepairDecisionTrace capture;
      require(fpga::repairNetRouteGaps(s.net,0),"dense-owner gap repair failed");
      decisions=capture.read();
    }
    check_complete_path(f,route,17);
    if(shared) check_complete_path(f,s.sinks[1].wires.front(),17);
    std::ostringstream owners_after;
    for(int n=0;n<32;++n) fpga::dumpNetRouteHistory(f.signal(910000+n).net,owners_after);
    require(owners_before.str()==owners_after.str(),"gap repair changed a foreign transit owner");
    f.state();
    size_t queries=0, at=0;
    while((at=decisions.find("REPAIR verify_shared_src=",at))!=std::string::npos) {++queries; ++at;}
    size_t source_queries=0; at=0;
    while((at=decisions.find("REPAIR verify_source_owner=",at))!=std::string::npos) {++source_queries; ++at;}
    std::printf("dense-owner gap repair: shared=%d source_only=%d complete=1 owner_queries=%zu source_queries=%zu\n",
        shared,source_only,queries,source_queries);
    require(shared ? queries>0 && queries<=16 : queries==0,
            "BUG: busy foreign SRCs trigger full owner scans without a reusable same-source arc");
    require(source_only ? source_queries>0 && source_queries<=8 : source_queries==0,
            "BUG: whole-source repair scans foreign owners absent from its known source graph");
  }
}

// Reproduce the next recorded failure: a reached, free DST has a direct arc
// to this binding's own sink, but the old two-JOINT input remains protected.
// Numeric coordinates/arcs are captured; all textual names are synthetic.
enum class SinkGuard { None, Shared, Alias, Foreign };
static bool run_private_grounding_replay(bool retain_grounding, SinkGuard guard=SinkGuard::None,
                                         bool retain_joints=false) {
  Fixture f(42);
  auto gap=[](Coord from,int dst,Coord to,int kind,int node) {
    Wire w; w.type=Wire::WIRE_GAP; w.from=from; w.to=to;
    w.from_node_type=fpga::CB_NODE_DST; w.from_node=dst;
    w.to_node_type=kind; w.to_node=node; return w;
  };
  Route r{f.pin({22,32},156,true),f.edge({22,32},{22,33},156,19,22,0),
      f.edge({22,33},{16,33},22,3079,3075),
      f.edge({16,33},{12,33},3075,3079,3075),
      f.edge({12,33},{5,37},3075,3379,3379),
      f.edge({5,37},{4,37},3379,3079,1027),
      f.edge({4,37},{11,37},1027,1030,1027),
      f.edge({11,37},{15,33},1027,979,979),
      f.edge({15,33},{15,31},979,195,195),
      f.edge({15,31},{15,30},195,244,248),
      f.edge({15,30},{15,29},248,249,244)};
  r.back().owns_landing=true;
  r.push_back(gap({15,29},244,{11,22},fpga::CB_NODE_DST,3538));
  r.push_back(f.edge({11,22},{11,21},3538,248,242));
  r.back().owns_landing=true;
  if (retain_grounding) {
    r.push_back(gap({11,21},242,{16,21},fpga::CB_NODE_DST,1024));
    r.push_back(f.edge({16,21},{16,19},1024,192,192));
    r.push_back(f.edge({16,19},{22,23},192,816,816));
    r.push_back(f.edge({22,23},{21,24},816,3616,3616));
    r.push_back(f.edge({21,24},{15,29},3616,3376,3376));
    r.push_back(f.edge({15,29},{11,33},3376,3376,3376));
    r.push_back(f.edge({11,33},{11,39},3376,96,96));
    r.push_back(f.edge({11,39},{11,40},96,20,24));
    r.push_back(f.edge({11,40},{12,40},24,258,262));
    r.push_back(f.edge({12,40},{15,39},262,738,738));
    r.push_back(f.edge({15,39},{15,37},738,194,194));
    r.push_back(f.edge({15,37},{12,37},194,3843,3848));
    f.terminal(r,{12,37},3848,111,11,25);
    require(r.size()==27,"private Grounding replay must retain the captured 27 fragments");
  } else {
    // Separate positive control: same source/sink and source prefix, but no
    // retained terminal input. Keep the sink pin leased in BOTH fixtures.
    r.push_back(gap({11,21},242,{12,37},fpga::CB_NODE_LOCAL,111));
    r.push_back(f.pin({12,37},111,false));
  }
  // A complete sibling keeps the four-hop source prefix live independently.
  Route sibling(r.begin(),r.begin()+5);
  for(auto &w:sibling) w.shared=true;
  f.terminal(sibling,{5,37},3379,3100);
  auto &route=f.install(162316,std::move(r));
  f.install(162316,std::move(sibling));
  auto &s=f.signal(162316);
  if (guard!=SinkGuard::None) {
    Route preserved=route;
    for (auto &w:preserved) {
      w.shared=true; w.owns_landing=false;
    }
    // Negative control: a separate complete path conflicts ONLY at the sink,
    // not a copy of the entire source tree under another driver's identity.
    if (guard==SinkGuard::Foreign) {
      preserved={f.pin({13,37},3004,true),f.edge({13,37},{12,37},3004,3004,3005,0)};
      f.terminal(preserved,{12,37},3005,111);
    }
    const int id=guard==SinkGuard::Shared?162316:162317;
    f.install(id,std::move(preserved));
    if (guard==SinkGuard::Alias) f.signal(id).net.routes.front().from=&s.driver;
  }
  // The deliberate foreign-owner corruption is exactly one conflicting
  // LOCAL. All positive fixtures must have zero driver conflicts.
  auto state=[&] { return f.state(guard==SinkGuard::Foreign?1:0); };

  // Last two hops and intermediate LOCALs of the actual six-hop alternative.
  f.name({5,37},fpga::CB_NODE_LOCAL,128);
  f.name({5,37},fpga::CB_NODE_LOCAL,184);
  f.tile({5,37}).cb_type->dst_local[3379].local.setBit(128);
  f.tile({5,37}).cb_type->local_local[128].local.setBit(184);
  f.edge({5,37},{11,37},184,3078,1030,0);
  f.edge({11,37},{12,37},1030,260,263);
  auto &sink=f.tile({12,37});
  auto final_arc=[&](bool enable) {
    auto &mask=retain_joints?sink.cb_type->dst_joint[263].joint:sink.cb_type->dst_local[263].local;
    const int node=retain_joints?25:111;
    if (enable) mask.setBit(node); else mask.clearBit(node);
  };
  final_arc(true);
  f.rebuild();
  require(std::count_if(route.begin(),route.end(),[](const Wire &w) {
    return w.type==Wire::WIRE_GAP;
  })==2,"private Grounding replay must retain two gaps");
  const auto owners=fpga::findNetOwnersByNode(sink,fpga::CB_NODE_LOCAL,111);
  require(owners.size()==(guard==SinkGuard::Foreign?2u:1u) &&
          std::any_of(owners.begin(),owners.end(),[&](const auto &owner) {
            return owner.net==&s.net && owner.binding_index==0;
          }) &&
          sink.isPinNodeLeased(111) && !sink.cb.dst.jump.testBit(263) &&
          !f.tile({11,37}).cb.src.jump.testBit(260),
          "captured private sink ownership/free approach differs from fixture");
  const auto before=state();
  std::ostringstream strict,relaxed;
  const bool strict_reachable=fpga::auditNetRouteReachability(s.net,0,strict);
  require(strict_reachable==(guard==SinkGuard::None),
          "strict reachability disagrees with private/shared terminal policy");
  require(fpga::auditNetRouteReachability(s.net,0,relaxed,false)==(guard!=SinkGuard::Foreign) &&
          relaxed.str().find("limited=1")==std::string::npos,
          "independent audit missed the captured direct completion");
  // Without the direct final arc even relaxed input rules cannot reach it.
  final_arc(false);
  std::ostringstream broken;
  require(!fpga::auditNetRouteReachability(s.net,0,broken,false),
          "fixture contains an accidental alternate sink path");
  require(!fpga::repairNetRouteGaps(s.net,0) && state()==before,
          "repair crossed a missing final arc or changed failed routing");
  final_arc(true);
  require(state()==before,"read-only topology probes changed leases or routes");
  require(!fpga::repairNetRouteGaps(s.net,0,[]{return true;}) && state()==before,
          "cancelled private sink repair changed routing");
  require(!fpga::repairNetRouteGaps(s.net,0,{},32768,5) && state()==before,
          "private sink repair bypassed the total hop bound");
  auto &bridge=f.tile({5,37});
  bridge.input_joint_reservations_initialized=true;
  bridge.input_local_reservations.emplace(128,nullptr);
  const auto reserved=state();
  require(!fpga::repairNetRouteGaps(s.net,0) && state()==reserved,
          "private sink exception leaked into an intermediate packed LOCAL");
  bridge.input_local_reservations.erase(128);

  SourceRepairTrace trace(s.net.name);
  pnr::RouteDesign router; router.fpga=&f.device; router.fanout_stage=true;
  pnr::RouteDesign::RouteTask task{.from=&s.driver,.to=&s.sinks[0],.net=&s.net,
      .from_port="out",.to_port="in",.net_name=s.net.routes[0].route_name,
      .fanout=true,.endpoints_prepared=true};
  const bool repaired=router.routeNetTask(task);
  require(router.pending_route_todo.empty(),"private sink replacement preempted another route");
  if (!repaired) {
    require(guard!=SinkGuard::None && !strict_reachable && state()==before,
            "private sink still rejected, or protected sink rejection changed routing");
    std::ifstream in(trace.path); std::string line; bool reached=false, limited=false;
    while(std::getline(in,line)) {
      if(line.starts_with("SOURCE_SEARCH_LIMIT ")) limited=true;
      if(line.starts_with("SOURCE_SEARCH_NODE ") &&
         line.find("tile=12,37 kind="+std::to_string(fpga::CB_NODE_DST)+" node=263 ")!=std::string::npos &&
         line.find("hops=6 expanded=1")!=std::string::npos) reached=true;
    }
    require(reached && !limited,"production did not reach and expand the recorded free six-hop arrival");
    require(!fpga::repairNetRouteGaps(s.net,0,{},65536,200) && state()==before,
            "larger budget changed the private-terminal failure");
    std::printf("private Grounding: preserved guard=%d rejected unchanged; reached six-hop DST263; ownership clean\n",int(guard));
  } else {
    require(guard!=SinkGuard::Foreign,"repair rewrote a foreign terminal input");
    check_complete_path(f,route,6);
    require(route[route.size()-2].local==263 &&
            route[route.size()-2].joint==(retain_joints?11:-1) &&
            route[route.size()-2].joint2==(retain_joints?25:-1) &&
            route.back().local==111 && sink.isPinNodeLeased(111),
            "replacement did not use the captured direct arrival and preserve the sink pin");
    require(!sink.cb.dst.jump.testBit(3848) && sink.cb.joint.jump.testBit(25)==retain_joints &&
            sink.cb.joint.jump.testBit(11)==retain_joints,"replacement leaked or lost private Grounding resources");
    if(guard==SinkGuard::Shared) check_complete_path(f,s.sinks[2].wires.front(),6);
    if(guard==SinkGuard::Alias) check_complete_path(f,f.signal(162317).sinks[0].wires.front(),6);
    std::printf("private Grounding: retain_old=%d six-hop completion, no preemption, pin retained\n",retain_grounding);
  }
  check_complete_path(f,s.sinks[1].wires.front(),4);
  state();
  return repaired;
}

void fanout_private_grounding_regression() {
  // Re-enter the OLD two-JOINT tail from a new DST. Physical ownership alone
  // is insufficient: same-source interrupted replicas (including aliases)
  // must be replaced together, including those that do not own a lease.
  // Foreign terminal users remain protected.
  for (auto guard:{SinkGuard::None,SinkGuard::Shared,SinkGuard::Alias,SinkGuard::Foreign})
    require(run_private_grounding_replay(true,guard,true)==(guard!=SinkGuard::Foreign),
            "private/shared terminal JOINT replacement policy was violated");
  require(run_private_grounding_replay(true,SinkGuard::Shared),"repair did not update the interrupted sibling");
  require(run_private_grounding_replay(true,SinkGuard::Alias),"repair did not update the interrupted alias");
  require(!run_private_grounding_replay(true,SinkGuard::Foreign),"repair consumed a foreign-owned sink");
  const bool repaired=run_private_grounding_replay(true);
  require(run_private_grounding_replay(false),"same leased sink without old Grounding could not route");
  require(repaired,"BUG: fanout repair cannot replace its own private two-JOINT Grounding with the reached direct input");
}

static void retained_suffix_node_cycle() {
  Fixture f;
  const Coord source{10,10}, trapped{11,10}, turn{12,10}, sink{13,10};
  Route r{f.pin(source,100,true),f.edge(source,trapped,100,90,90,0)};
  r.back().owns_landing=true;
  Wire gap; gap.type=Wire::WIRE_GAP; gap.from=trapped; gap.to=source;
  gap.from_node_type=fpga::CB_NODE_DST; gap.from_node=90;
  gap.to_node_type=fpga::CB_NODE_LOCAL; gap.to_node=100;
  r.push_back(gap);
  r.push_back(f.edge(source,turn,100,110,111,0));
  r.push_back(f.edge(turn,source,111,112,113));
  r.back().owns_landing=true;
  Wire back; back.type=Wire::WIRE_ROUTE_EDGE; back.from=back.to=source;
  back.from_node_type=fpga::CB_NODE_DST; back.from_node=113;
  back.to_node_type=fpga::CB_NODE_LOCAL; back.to_node=100;
  r.push_back(back);
  f.tile(source).cb_type->dst_local[113].local.setBit(100);
  r.push_back(f.edge(source,sink,100,114,115,0));
  f.terminal(r,sink,115,116);
  auto &route=f.install(732,std::move(r)); f.rebuild();
  const auto before=f.state();
  // One state reaches the source, which occurs twice in the retained tail.
  // Appending that tail would make a cycle. There is no budget to explore
  // the legitimate short path yet, so failure must leave everything intact.
  require(!fpga::repairNetRouteGaps(f.signal(732).net,0,{},1,200) && f.state()==before,
          "retained suffix introduced a repeated source node into the prefix");
  require(fpga::repairNetRouteGaps(f.signal(732).net,0,{},128,200),
          "rejecting a cyclic suffix prevented a separate loop-free completion");
  check_complete_path(f,route,1); f.state();
}

static void retained_suffix_tile_repetition(bool third_visit) {
  Fixture f;
  const Coord source{10,10}, trapped{11,10}, join{12,10}, middle{14,10}, sink{15,10};
  const Coord last=third_visit?source:Coord{10,11};
  Route r{f.pin(source,100,true),f.edge(source,trapped,100,100,101,0)};
  r.back().owns_landing=true;
  Wire gap; gap.type=Wire::WIRE_GAP; gap.from=trapped; gap.to=join;
  gap.from_node_type=gap.to_node_type=fpga::CB_NODE_DST;
  gap.from_node=101; gap.to_node=201;
  r.push_back(gap);
  r.push_back(f.edge(join,source,201,201,202));
  r.push_back(f.edge(source,middle,202,202,203));
  r.push_back(f.edge(middle,last,203,203,204));
  r.push_back(f.edge(last,sink,204,204,205));
  f.terminal(r,sink,205,206,19,20);
  auto &route=f.install(731,std::move(r));
  // The exact gap is sealed; only a different source exit reaches the tail.
  f.edge(source,join,100,111,201,0); f.rebuild();
  const auto before=f.state();
  if (!third_visit) {
    auto &terminal=f.tile(sink);
    terminal.cb_type->joint_joint[20].joint.clearBit(19);
    require(!fpga::repairNetRouteGaps(f.signal(731).net,0,{},128,200) && f.state()==before,
            "suffix shortcut crossed a missing second-JOINT connection");
    terminal.cb_type->joint_joint[20].joint.setBit(19);
    terminal.input_joint_reservations_initialized=true;
    NodeMask reserved; reserved.setBit(19);
    terminal.input_joint_reservations.emplace_back(nullptr,reserved);
    const auto packed=f.state();
    require(!fpga::repairNetRouteGaps(f.signal(731).net,0,{},128,200) && f.state()==packed,
            "suffix shortcut crossed a packed-input JOINT reservation");
    terminal.input_joint_reservations.clear();
  }
  const bool repaired=fpga::repairNetRouteGaps(f.signal(731).net,0,{},128,200);
  if (third_visit) {
    require(!repaired && f.state()==before,
            "suffix reconnection admitted a third arrival at the source Tile");
  } else {
    require(repaired,"suffix reconnection rejected a legal second Tile arrival");
    check_complete_path(f,route,5); f.state();
  }
}

// Third failure: source search reaches a terminal-connected retained suffix,
// but spends its remaining state budget rediscovering that suffix. Preserve
// the recorded numeric route; replace unrelated chip fabric by dead-end load.
void fanout_retained_suffix_regression() {
  retained_suffix_node_cycle();
  retained_suffix_tile_repetition(true);
  retained_suffix_tile_repetition(false);
  Fixture f;
  Route r{f.pin({22,32},156,true),f.edge({22,32},{22,33},156,19,22,0),
      f.edge({22,33},{16,33},22,3079,3075),
      f.edge({16,33},{15,33},3075,3846,3842),
      f.edge({15,33},{15,31},3842,195,195),
      f.edge({15,31},{15,30},195,244,248),
      f.edge({15,30},{15,29},248,249,244),
      f.edge({15,29},{12,28},244,3810,3810),
      f.edge({12,28},{12,25},3810,194,194),
      f.edge({12,25},{12,19},194,162,162),
      f.edge({12,19},{16,15},162,978,978),
      f.edge({16,15},{16,14},978,248,242),
      f.edge({16,14},{15,13},242,3809,3809),
      f.edge({15,13},{12,14},3809,3616,3616),
      f.edge({12,14},{12,15},3616,16,19)};
  f.terminal(r,{12,15},19,80);
  require(r.size()==17,"retained-suffix original route must have 17 fragments");
  Route sibling(r.begin(),r.begin()+3);
  for (auto &w:sibling) w.shared=true;
  f.terminal(sibling,{16,33},3075,86);
  auto &route=f.install(162316,std::move(r));
  f.install(162316,std::move(sibling));
  auto &s=f.signal(162316);
  f.rebuild(); f.state();
  const std::vector<fpga::RouteCutNode> cuts{
      {{15,29},fpga::CB_NODE_SRC,3810},{{12,19},fpga::CB_NODE_DST,162}};
  const auto victims=fpga::findNetRouteCutVictims(cuts);
  require(victims.size()==1 && victims.front().binding_index==0 &&
          fpga::unrouteNetRoutesFromNodes(victims,cuts,true),
          "could not reproduce the retained-suffix gap by real preemption");
  require(std::count_if(route.begin(),route.end(),[](const Wire &w) {
    return w.type==Wire::WIRE_GAP;
  })==1,"retained-suffix replay must contain exactly one gap");
  for (const auto &w:route) if(w.type==Wire::WIRE_GAP)
    check_gap(w,{15,29},244,{16,15},978,fpga::CB_NODE_DST);
  require(fpga::isRouteComplete(s.sinks[1].wires.front()),"cut damaged completed sibling");

  // All eight blocked exact-gap exits have real foreign route owners.
  f.owner(161940,{15,27},194,900,{17,27},900,true);
  f.owner(162972,{15,29},901,243,{17,29},901,true);
  f.owner(164144,{15,29},902,248,{17,30},902,true);
  f.owner(163027,{15,29},903,259,{17,31},903,false,true);
  f.owner(164043,{16,28},738,904,{18,28},904,true);
  f.owner(165022,{15,29},905,1028,{18,29},905,true);
  f.owner(162708,{15,29},906,3810,{12,28},3810,true);
  f.owner(166407,{15,29},907,3843,{18,31},907,true);
  for(int src:{243,248,259,1028,3810,3843})
    f.tile({15,29}).cb_type->dst_src[244].jump.setBit(src);
  f.edge({15,29},{16,28},244,738,738);
  f.edge({15,29},{15,27},244,194,194);

  // The nine-hop source-to-gap-end path actually reached in the failure.
  f.edge({16,33},{21,32},3075,262,260);
  f.tile({21,32}).cb_type->dst_local[260].local.setBit(178);
  f.edge({21,32},{11,32},178,3072,3077,0);
  f.tile({11,32}).cb_type->dst_local[3077].local.setBit(196);
  f.edge({11,32},{4,28},196,3539,3539,0);
  f.edge({4,28},{4,21},3539,163,163);
  f.tile({4,21}).cb_type->dst_local[163].local.setBit(165);
  f.edge({4,21},{12,21},165,3078,1030,0);
  f.edge({12,21},{12,19},1030,194,194);
  f.edge({12,19},{16,15},194,978,978);

  // Synthetic background load, NOT claimed to reproduce the full chip.
  // At the recorded search costs these eight landings have priority 75:
  // after the retained second-hop arrival is queued, before it is expanded
  // (priority 79). Their free, terminal-less LOCALs fill the default 32768
  // state budget. No arbitrary busy bits or reduced production limits.
  const std::array<Coord,8> load_tiles{{{12,9},{11,10},{10,11},{9,12},
                                     {8,13},{7,14},{6,15},{7,16}}};
  for (size_t i=0;i<load_tiles.size();++i) {
    const auto at=load_tiles[i];
    f.edge({16,15},at,978,100+i,100);
    for (int n=0;n<CB_MAX_NODES;++n) {
      f.name(at,fpga::CB_NODE_LOCAL,n);
      f.tile(at).cb_type->dst_local[100].local.setBit(n);
    }
  }
  f.rebuild();
  const auto before=f.state();
  auto other_routes=[&] {
    std::ostringstream out;
    for (const auto &[id,signal]:f.signals) if(id!=162316)
      fpga::dumpNetRouteHistory(signal->net,out);
    // Successful replacement may promote the sibling to primary lease owner.
    // Its physical path must stay identical; f.state() checks the new leases.
    for(const auto &w:s.sinks[1].wires.front())
      out << int(w.type) << ':' << w.from.x << ',' << w.from.y << ':'
          << w.to.x << ',' << w.to.y << ':' << w.local << ':' << w.jump << ':'
          << w.dst << ':' << w.pos << ':' << w.joint << ':' << w.joint2 << ':'
          << w.from_node_type << ':' << w.from_node << ':' << w.to_node_type << ':'
          << w.to_node << ':' << w.pin_dir << ';';
    return out.str();
  };
  const auto others=other_routes();
  std::ostringstream audit;
  require(fpga::auditNetRouteReachability(s.net,0,audit) &&
          audit.str().find("limited=1")==std::string::npos,
          "independent input-preserving audit could not reach the sink");
  f.tile({12,15}).cb_type->dst_local[19].local.clearBit(80);
  std::ostringstream broken;
  require(!fpga::auditNetRouteReachability(s.net,0,broken),
          "fixture has an accidental shortcut around the retained suffix");
  require(!fpga::repairNetRouteGaps(s.net,0) && f.state()==before,
          "repair trusted stored fragments across a physically broken suffix");
  f.tile({12,15}).cb_type->dst_local[19].local.setBit(80);
  require(f.state()==before,"read-only reachability controls changed routing");
  require(!fpga::repairNetRouteGaps(s.net,0,{},32768,12) && f.state()==before,
          "suffix shortcut bypassed the full source-to-sink hop limit");
  require(!fpga::repairNetRouteGaps(s.net,0,[]{return true;}) && f.state()==before,
          "cancelled suffix repair changed routing");

  SourceRepairTrace trace(s.net.name);
  pnr::RouteDesign router; router.fpga=&f.device; router.fanout_stage=true;
  pnr::RouteDesign::RouteTask task{.from=&s.driver,.to=&s.sinks[0],.net=&s.net,
      .from_port="out",.to_port="in",.net_name=s.net.routes[0].route_name,
      .fanout=true,.endpoints_prepared=true};
  const bool repaired=router.routeNetTask(task);
  if (!repaired) {
    require(f.state()==before && other_routes()==others && router.pending_route_todo.empty(),
            "failed repair changed retained routes, owners or pending tasks");
    std::ifstream in(trace.path); std::ostringstream contents; contents << in.rdbuf();
    const auto log=contents.str();
    auto node=[&](const char *at,int kind,int id,const char *tail) {
      return std::string("tile=")+at+" kind="+std::to_string(kind)+" node="+std::to_string(id)+tail;
    };
    require(log.find("SOURCE_SEARCH end states=32768")!=std::string::npos &&
            log.find(node("16,15",fpga::CB_NODE_DST,978," cost=57 hops=9 expanded=1"))!=std::string::npos &&
            log.find(node("15,13",fpga::CB_NODE_DST,3809," cost=69 hops=11 expanded=1"))!=std::string::npos &&
            log.find(node("15,13",fpga::CB_NODE_SRC,3616," cost=70 hops=11"))!=std::string::npos,
            "failure did not match reached suffix plus state-limit rejection two hops before sink");
    std::puts("retained suffix: reached (16,15)/DST978 at hop 9; reached (15,13)/DST3809 at hop 11;"
              " state cap=32768 rejects SRC3616 with two hops left; ownership unchanged");
    // Control only: prove the exact same fabric completes with more budget.
    // This is not the requested behavior and not a production-limit change.
    require(fpga::repairNetRouteGaps(s.net,0,{},65536,200),
            "higher-budget control failed despite a verified free completion");
  }
  require(fpga::isRouteComplete(route),"repair did not produce a complete route");
  check_complete_path(f,route,13);
  require(other_routes()==others && router.pending_route_todo.empty() &&
          fpga::isRouteComplete(s.sinks[1].wires.front()),
          "repair changed a completed sibling path or foreign owner");
  f.state();
  std::puts("retained suffix: complete 13-hop path verified; sibling and foreign owners unchanged");
  require(repaired,"BUG: default-budget fanout repair rejected a reached, intact four-hop retained suffix");
  std::ifstream in(trace.path); std::ostringstream contents; contents << in.rdbuf();
  const auto marker=std::string("SOURCE_SEARCH_SUFFIX tile=16,15 kind=")+
      std::to_string(fpga::CB_NODE_DST)+" node=978 prefix_hops=9 suffix_hops=4 total_hops=13 states=";
  const auto at=contents.str().find(marker);
  require(at!=std::string::npos,"repair did not reconnect at the recorded retained-suffix start");
  const auto states=std::stoul(contents.str().substr(at+marker.size()));
  require(states<32768,"repair unnecessarily exhausted the fabric state budget");
  std::printf("retained suffix: default-budget success; states=%zu; broken-tail, hop-limit and Tile-repetition controls passed\n",states);
}

// Retain the recorded 14-fragment shape and two-JOINT input. The unrecorded
// fabric is synthetic: a broad dead-end region competes with a short, free
// source-to-input bypass. This tests repair starvation, not device reachability.
void fanout_gap_budget_regression(bool failure_location, bool wide_probe) {
  Fixture f;
  Route r{f.pin({11,6},149,true),
      f.edge({11,6},{15,6},149,1024,1024,0),
      f.edge({15,6},{16,7},1024,544,544),
      f.edge({16,7},{16,8},544,16,19),
      f.edge({16,8},{17,8},19,901,910),
      f.edge({17,8},{5,6},910,911,3537),
      f.edge({5,6},{11,6},3537,902,1029),
      f.edge({11,6},{15,6},1029,1026,1025),
      f.edge({15,6},{15,4},1025,193,193),
      f.edge({15,4},{15,3},193,903,99),
      f.edge({15,3},{15,4},99,904,24),
      f.edge({15,4},{12,5},24,905,3619),
      f.edge({12,5},{12,7},3619,67,67)};
  f.terminal(r,{12,7},67,107,45,60);
  auto &route=f.install(741,std::move(r));
  require(route.size()==15 && fpga::isRouteComplete(route),
          "budget replay did not start with the recorded complete shape");
  require(fpga::unrouteNetRoutesFromNodes({{&f.signal(741).net,0}},
      {{{16,8},fpga::CB_NODE_SRC,901},{{17,8},fpga::CB_NODE_DST,910}},true),
      "could not preempt the replay transit");
  require(route.size()==14,"preempted route must have fourteen fragments");
  check_gap(route[4],{16,8},19,{5,6},3537,fpga::CB_NODE_DST);
  // A real Takeoff owner now holds the cut resource. No naked occupied bits.
  f.owner(742,{16,8},150,901,{17,8},910,false,true);
  for (int n=1000;n<(wide_probe?3000:1256);++n) {
    f.edge({16,8},{20,20},19,n,n);
    f.name({20,20},fpga::CB_NODE_LOCAL,n);
    f.tile({20,20}).cb_type->dst_local[n].local.setBit(n);
  }
  if(wide_probe) {
    // A 1,024-state source probe fills its frontier before admitting the
    // useful exit. The subsequent exact-gap flood has no path to its target.
    // Model the measured small-probe/flood/full-source-repair sequence using
    // deterministic work counts, not a machine-dependent elapsed-time limit.
    for(int n=1000;n<2400;++n) if(n!=1024 && n!=1026)
      f.edge({11,6},{24,24},149,n,n,0);
  }
  f.edge({11,6},{12,7},149,wide_probe?3500:906,907,0);
  f.tile({12,7}).cb_type->dst_local[907].local.setBit(107);
  f.rebuild();
  auto &s=f.signal(741);
  require(f.device.resolveJumpTargets(f.tile({16,8}),1000).size()==1,
          "background load has no physical landing");
  const auto before=f.state();
  if (failure_location) {
    pnr::RouteDesign router; router.fpga=&f.device; router.fanout_stage=true;
    pnr::RouteDesign::RouteTask task{.from=&s.driver,.to=&s.sinks[0],.net=&s.net,
        .from_port="out",.to_port="in",.net_name=s.net.routes[0].route_name,
        .fanout=true,.endpoints_prepared=true};
    router.route_stage_deadline_enabled=true;
    router.route_stage_deadline=std::chrono::steady_clock::now()-std::chrono::seconds(1);
    require(!router.routeNetTask(task) && router.route_stage_deadline_expired &&
            f.state()==before,"cancelled production repair changed routing or lost deadline status");
    std::printf("gap cancellation: reported=(%d,%d) kind=%d node=%d\n",
        task.failure_coord.x,task.failure_coord.y,int(task.failure_node_type),task.failure_node);
    require(same(task.failure_coord,{16,8}) && task.failure_node_type==fpga::CB_NODE_DST &&
            task.failure_node==19,"BUG: cancelled gap repair was reported at the retained sink's invalid DST");
    return;
  }
  unsigned polls=0;
  const unsigned work_budget=wide_probe?2500:200;
  require(!fpga::repairNetRouteGaps(s.net,0,[]{return true;}) && f.state()==before,
          "cancelled repair modified the retained route");
  require(!fpga::repairNetRouteGaps(s.net,0,{},1,200) && f.state()==before,
          "repair bypassed the caller's state limit");
  require(!fpga::repairNetRouteGaps(s.net,0,{},32768,0) && f.state()==before,
          "repair bypassed the caller's hop limit");
  auto &sink_cb=*f.tile({12,7}).cb_type;
  sink_cb.dst_local[907].local.clearBit(107); f.rebuild();
  require(!fpga::repairNetRouteGaps(s.net,0,[&]{return ++polls>work_budget;}) && f.state()==before,
          "unreachable source probe changed routes or their owners");
  sink_cb.dst_local[907].local.setBit(107); f.rebuild();
  std::ostringstream owner_before;
  fpga::dumpNetRouteHistory(f.signal(742).net,owner_before);
  polls=0;
  // A deterministic work deadline, not a flaky wall-clock assertion. The
  // old FIFO gap search consumes this budget before trying the source path.
  const bool repaired=fpga::repairNetRouteGaps(s.net,0,[&]{return ++polls>work_budget;});
  std::printf("gap budget: success=%d cancellation_polls=%u\n",repaired,polls);
  require(repaired,wide_probe?"BUG: undersized source probe resumed a fruitless exact-gap flood":
          "BUG: detached-gap flood starved an available one-hop source repair");
  check_complete_path(f,route,1);
  require(fpga::isRouteComplete(f.signal(742).sinks.front().wires.front()),
          "repair damaged the preempting owner");
  std::ostringstream owner_after;
  fpga::dumpNetRouteHistory(f.signal(742).net,owner_after);
  require(owner_before.str()==owner_after.str(),"free-only repair changed the foreign owner");
  require(f.tile({12,7}).isPinNodeLeased(107),"repair lost the sink pin");
  f.state();
}

void fanout_gap_replay_regression() {
  Fixture f;
  f.victim(); f.blockers(); f.rebuild();
  auto &s=f.signal(162539);
  for(auto &sink:s.sinks) require(fpga::isRouteComplete(sink.wires.front()),
                                "victim did not start fully routed");
  require(s.sinks[2].wires.front().size()==18,"initial victim path was not 18 fragments");
  const auto initially_reachable = source_reachability(f);
  require(initially_reachable.gap_end && initially_reachable.sink_tile,
          "source reachability missed the original complete path");
  std::ostringstream full_audit;
  require(fpga::auditNetRouteReachability(s.net,2,full_audit),
          "node-level audit missed the original complete path");
  f.state(); // All initial occupancy already has real owners.
  f.preempt(); f.rebuild();
  auto &route=s.sinks[2].wires.front();
  check_gap(route[6],{16,28},242,{15,25},241,fpga::CB_NODE_DST);
  check_gap(route[13],{12,14},256,{12,17},106,fpga::CB_NODE_LOCAL);
  for(size_t i=0;i<s.sinks.size();++i) require(
      fpga::isRouteComplete(s.sinks[i].wires.front())==(i!=2 && i!=4),
      "final sibling routing states differ from recorded snapshot");
  check_frontier(f);
  const auto before=f.state();
  require(!fpga::auditNetRouteReachability(s.net,2,full_audit) &&
          !fpga::auditNetRouteReachability(s.net,2,full_audit,false),
          "node-level audit found an unexpected captured bypass");
  require(full_audit.str().find("limited=1")==std::string::npos,
          "node-level audit hit its limit instead of exhausting the graph");
  const auto reachable = source_reachability(f);
  require(!reachable.gap_end && !reachable.sink_tile,
          "captured topology now contains a possible wider free bypass");
  require(f.state()==before,"read-only source reachability changed routing");
  std::printf("gap replay: optimistic whole-source search exhausted %zu states; "
              "gap end unreachable; sink Tile unreachable\n", reachable.states);
  std::set<std::pair<int,int>> audited;
  require(!fpga::repairNetRouteGaps(s.net,2,{},32768,200,[&](Tile &t) {
    audited.emplace(t.coord.x,t.coord.y);
  }),"recorded trapped gap unexpectedly repaired");
  require(audited==std::set<std::pair<int,int>>{{16,28},{15,25},{22,28},{26,28},{16,32},{21,28}},
          "production repair visited a different diagnostic frontier");
  require(f.state()==before,"failed repair changed a route, lease or owner");
  require(!fpga::repairNetRouteGaps(s.net,2,{},65536,400),
          "larger budgets changed a genuinely exhausted frontier");
  pnr::RouteDesign router; router.fpga=&f.device; router.fanout_stage=true;
  pnr::RouteDesign::RouteTask task{.from=&s.driver,.to=&s.sinks[2],.net=&s.net,
      .from_port="out",.to_port="in",.net_name=s.net.routes[2].route_name,
      .fanout=true,.endpoints_prepared=true};
  require(!router.routeNetTask(task),"production fanout entry did not reproduce failed gap repair");
  require(f.state()==before,"production failure changed retained routing");
  require(same(task.failure_coord,{16,28}) &&
          task.failure_node_type==fpga::CB_NODE_DST && task.failure_node==242,
          "failed repair did not report its first unresolved gap boundary");
  std::puts("gap replay: reproduced two reached states, 20 busy SRCs, 3 busy landings; production fanout failed; ownership audit clean");

  const auto original_routes=f.unaffected_routes();
  f.real_bypasses();
  check_frontier(f); // The exact cut boundary remains trapped, as on the chip.
  const auto imported_state=f.state();
  std::ostringstream real_audit;
  if (!fpga::auditNetRouteReachability(s.net,2,real_audit))
    throw std::runtime_error("imported real source-to-sink bypass is not reachable\n" + real_audit.str());
  require(real_audit.str().find("SOURCE_GOAL goal=0 reached=1")!=std::string::npos &&
          real_audit.str().find("SOURCE_GOAL goal=1 reached=1")!=std::string::npos,
          "imported real bypass failed to reach both recorded goals");
  require(!fpga::repairNetRouteGaps(s.net,2,[]{return true;}) &&
          !fpga::repairNetRouteGaps(s.net,2,{},1) &&
          !fpga::repairNetRouteGaps(s.net,2,{},32768,9),
          "cancelled or budget-limited source repair succeeded");
  require(f.state()==imported_state && f.unaffected_routes()==original_routes,
          "limited repair modified retained routes or occupancy");
  // Remove each recorded intra-CB intermediate connection in turn: the
  // corresponding path must disappear. This excludes accidental shortcuts
  // introduced by the import or by shortened foreign owner routes.
  f.tile({25,34}).cb_type->dst_local[819].local.clearBit(165);
  std::ostringstream without_gap_bridge;
  require(fpga::auditNetRouteReachability(s.net,2,without_gap_bridge) &&
          without_gap_bridge.str().find("SOURCE_GOAL goal=0 reached=0")!=std::string::npos,
          "gap bypass did not require its real intermediate LOCAL");
  f.tile({25,34}).cb_type->dst_local[819].local.setBit(165);
  f.tile({15,25}).cb_type->dst_local[3077].local.clearBit(178);
  std::ostringstream without_sink_bridge;
  require(!fpga::auditNetRouteReachability(s.net,2,without_sink_bridge) &&
          without_sink_bridge.str().find("SOURCE_GOAL goal=0 reached=1")!=std::string::npos,
          "sink bypass did not require its real intermediate LOCAL");
  require(!router.routeNetTask(task),"repair crossed a missing intermediate LOCAL arc");
  f.tile({15,25}).cb_type->dst_local[3077].local.setBit(178);
  require(f.state()==imported_state,"LOCAL bridge probes changed occupancy");
  // A physically free LOCAL is not usable if placement reserved it as an
  // input. The imported wire itself has no such reservation in the snapshot.
  auto &bridge_tile=f.tile({15,25});
  bridge_tile.input_joint_reservations_initialized=true;
  bridge_tile.input_local_reservations.emplace(178,nullptr);
  const auto reserved_state=f.state();
  std::ostringstream packed_audit;
  require(!fpga::auditNetRouteReachability(s.net,2,packed_audit),
          "reachability audit ignored a packed intermediate LOCAL reservation");
  require(!router.routeNetTask(task) && f.state()==reserved_state,
          "source repair consumed a packed input as an intermediate wire");
  bridge_tile.input_local_reservations.erase(178);
  require(router.routeNetTask(task) && fpga::isRouteComplete(route),
          "production repair failed to use the verified real source-to-sink bypass");
  require(std::any_of(route.begin(),route.end(),[](const Wire &w) {
    return w.type==Wire::WIRE_ROUTE_EDGE && same(w.from,{15,25}) &&
        w.from_node_type==fpga::CB_NODE_DST && w.from_node==3077 &&
        w.to_node_type==fpga::CB_NODE_LOCAL && w.to_node==178;
  }),"replacement omitted the real intermediate LOCAL connection");
  require(f.tile({15,25}).cb.local.local.testBit(178) &&
          !f.tile({15,25}).isPinNodeLeased(178),
          "intermediate LOCAL not leased as a routing wire");
  require(fpga::hasRouteGaps(s.sinks[4].wires.front()),"repair modified independent partial sibling");
  require(f.unaffected_routes()==original_routes,"source repair changed unrelated retained routes");
  check_complete_path(f,route);
  f.state();
  std::puts("gap replay: production repaired both gaps through the real ten-hop bypass; foreign and sibling owners retained");
  // The intermediate LOCAL bridge is transit, not an unpreemptible endpoint.
  for (const auto &cut:std::vector<fpga::RouteCutNode>{
      {{15,25},fpga::CB_NODE_SRC,3376}, {{15,25},fpga::CB_NODE_LOCAL,178},
      {{15,25},fpga::CB_NODE_DST,3077}, {{11,30},fpga::CB_NODE_DST,3376}}) {
    auto victims=fpga::findNetRouteCutVictims({cut});
    require(victims.size()==1 && victims.front().net==&s.net && victims.front().binding_index==2,
            "bridge transit has incorrect preemption dependencies");
    require(!fpga::previewNetRouteCuts(victims,{cut},true).empty() &&
            fpga::unrouteNetRoutesFromNodes(victims,{cut},true),
            "repaired LOCAL bridge cannot be partially preempted");
    require(fpga::hasRouteGaps(route) && !f.tile({15,25}).cb.local.local.testBit(178),
            "bridge preemption left a complete route or leaked its LOCAL");
    require(f.unaffected_routes()==original_routes,"bridge preemption changed sibling or foreign routes");
    f.state();
    require(router.routeNetTask(task),"preempted bridge could not be repaired again");
    check_complete_path(f,route);
    f.state();
  }
  for (const auto &cut:std::vector<fpga::RouteCutNode>{
      {{25,28},fpga::CB_NODE_SRC,1025}, {{12,17},fpga::CB_NODE_DST,246}})
    require(fpga::previewNetRouteCuts(fpga::findNetRouteCutVictims({cut}),{cut},true).empty(),
            "source repair made Takeoff or Grounding preemptible");
  require(fpga::unrouteNetRoute(s.net,2),"could not remove repaired binding");
  require(!f.tile({15,25}).cb.local.local.testBit(178),
          "unrouting leaked intermediate LOCAL ownership");
  f.state();
}
