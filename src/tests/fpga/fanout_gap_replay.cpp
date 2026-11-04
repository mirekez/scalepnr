#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"
#include "RepairStatistics.h"
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
#include <format>
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
  std::string node_pattern;

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
    if (!node_pattern.empty()) {
      tile(c).cb_type->rememberNodeName(kind, id,
          std::format("{}_x{:02}_y{:02}_k{}_n{:04}", node_pattern, c.x, c.y, int(kind), id));
      return;
    }
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
      if (kind==fpga::CB_NODE_LOCAL && next_kind==fpga::CB_NODE_JOINT)
        connected=cb.local_joint[id].joint.testBit(next_id);
      if (kind==fpga::CB_NODE_JOINT && next_kind==fpga::CB_NODE_SRC)
        connected=cb.joint_src[id].jump.testBit(next_id);
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

// Example 1's run-3 deadline audit found this exact retained pair:
//   fragment 3: (58,23)/DST3377 -> SRC258 -> (62,23)/DST262,
//               private FORK, owns_dst=true, owns_landing=false;
//   fragment 4: (62,23)/DST262 -> SRC260 -> (63,23)/DST263,
//               private FORK, owns_dst=false, owns_landing=false.
// DST262 had a live borrower but no lease/owner. The audit did not record
// which earlier edit removed its last owner. Exercise ordinary owner removal
// with that exact retained geometry; only the earlier prefix and donor tail
// are synthetic. A private prefix must not hide an island's borrowed landing.
// Every occupied bit comes from a registered route; names use our own pattern.
void fanout_private_island_landing_regression(bool reproduce) {
  for (int mode = 0; mode < 4; ++mode) {
    Fixture f(28, 70);
    f.node_pattern = std::format("island_owner_m{}", mode);
    constexpr int donor_id = 9750, alias_id = 9751;
    const bool alias = mode == 1, shared_prefix = mode == 2, no_borrower = mode == 3;
    const Coord source{57,23}, prefix_end{57,24}, incoming{58,23}, fork{62,23};
    auto root = f.edge(source,prefix_end,10,2010,2011,0);
    root.owns_landing = true;
    Wire hole;
    hole.type = Wire::WIRE_GAP;
    hole.from = prefix_end; hole.from_node_type = fpga::CB_NODE_DST; hole.from_node = 2011;
    hole.to = incoming; hole.to_node_type = fpga::CB_NODE_DST; hole.to_node = 3377;
    auto arrival = f.edge(incoming,fork,3377,258,262,2);
    arrival.owns_dst = true;
    arrival.owns_landing = false;
    Route prefix{f.pin(source,10,true),root,hole,arrival};
    Route donor = prefix;
    donor.push_back(f.edge(fork,{62,24},262,270,270));
    f.terminal(donor,{62,24},270,91);
    f.install(donor_id,std::move(donor));
    Route* borrower = nullptr;
    rtl::Net* borrower_net = nullptr;
    size_t borrower_binding = 0;
    if (!no_borrower) {
      Route branch = prefix;
      if (shared_prefix) {
        branch[0].shared = branch[1].shared = true;
        branch[1].owns_landing = false;
      }
      branch.push_back(f.edge(fork,{63,23},262,260,263,2));
      branch.push_back(f.edge({63,23},{66,22},263,262,260));
      f.terminal(branch,{66,22},260,90);
      borrower = &f.install(alias ? alias_id : donor_id,std::move(branch));
      borrower_net = &f.signal(alias ? alias_id : donor_id).net;
      borrower_binding = borrower_net->routes.size() - 1;
      if (alias) {
        borrower_net->routes[borrower_binding].from = &f.signal(donor_id).driver;
        borrower_net->invalidateRouteLookup();
      }
    }
    // A real free continuation makes the retained branch repairable after
    // removal. Its five inter-Tile hops must then survive independent checks.
    f.edge(prefix_end,incoming,2011,2012,3377);
    f.rebuild(); f.state();
    require(fpga::unrouteNetRoute(f.signal(donor_id).net,0),"cannot remove island donor");
    std::ostringstream proof;
    auto audit = fpga::auditTileCongestion(f.tile(fork),proof,
        {&f.signal(donor_id).net, alias ? borrower_net : nullptr});
    const bool leased = f.tile(fork).cb.dst.jump.testBit(262);
    const auto owners = fpga::findNetOwnersByNode(f.tile(fork),fpga::CB_NODE_DST,262);
    std::printf("ISLAND_OWNER mode=%d borrower=%d leased=%d owners=%zu missing=%zu\n",
                mode,!no_borrower,leased,owners.size(),audit.missing_leases);
    require(!audit.orphan_leases && !audit.missing_registrations &&
            !audit.stale_registrations && !audit.conflicting_driver_nodes,
            "donor removal introduced unrelated ownership/index corruption");
    if (reproduce && mode < 2) {
      require(!leased && owners.empty() && audit.missing_leases == 1,
              "fixture did not reproduce the recorded unowned fork input");
      continue;
    }
    require(leased == !no_borrower && audit.missing_leases == 0,
            "donor removal released a private retained island's borrowed DST262");
    if (!no_borrower) {
      require(!owners.empty(),"retained island lease has no real owning fragment");
      f.state();
      require(fpga::repairNetRouteGaps(*borrower_net,borrower_binding) &&
              fpga::isRouteComplete(*borrower),"retained island cannot repair its free gap");
      check_complete_path(f,*borrower,5); f.state();
      require(fpga::unrouteNetRoute(*borrower_net,borrower_binding),"cannot remove final island owner");
      require(!f.tile(fork).cb.dst.jump.testBit(262),"final island teardown leaked its incoming DST");
    }
    // Ordinary routing teardown leaves the placed driver's output pin
    // reserved. This fixture has no packer: release that placement-owned pin
    // separately before checking that all routing resources are now empty.
    require(!f.tile(source).cb.local.local.testBit(10),"final teardown leaked its source LOCAL");
    f.tile(source).pin_state.leased_nodes &= ~(NodeMask{0,1} << 10);
    f.state();
  }
}


// Example 1, run 5: both requester cells are fixed by earlier chunks. Its
// 50-fragment route has two holes. The core proves a 58-node bypass with a
// Takeoff cut at (71,2)/SRC816 + (76,6)/DST816 and a Grounding cut at
// (21,6)/DST3539. The final jump departs FREE (25,10)/SRC3539, outside the
// radius-5 window, and lands INSIDE it. Rejecting this entry loses Grounding
// even though no remote transit resource needs cutting. Keep the exact old
// route, all three recorded partial owners and the witnessed topology.
// Unrelated siblings are omitted; normalize the imported owners' shared
// metadata so this fixture owns every lease it uses. Names are patterned,
// generated locally, and independent of any device vendor.
void fanout_window_entry_regression(bool reproduce) {
  for(int mode=0;mode<4;++mode) {
    Fixture f(60,100); f.node_pattern=std::format("window_entry_m{}",mode);
    struct Row { int type; Coord from,to; int local,pos,src,dst,joint,joint2,
                 from_kind,from_node,to_kind,to_node,owns_dst,owns_landing; };
    auto import=[&](int id,std::initializer_list<Row> rows,bool normalize)->Route& {
      Route route;
      for(const auto &r:rows) {
        Wire w; w.type=static_cast<decltype(w.type)>(r.type);
        w.from=r.from; w.to=r.to; w.resource=r.from; w.local=r.local;
        w.pos=r.pos; w.jump=w.route_jump=r.src; w.dst=r.dst;
        w.joint=r.joint; w.joint2=r.joint2;
        w.from_node_type=static_cast<fpga::CBNodeNameType>(r.from_kind); w.from_node=r.from_node;
        w.to_node_type=static_cast<fpga::CBNodeNameType>(r.to_kind); w.to_node=r.to_node;
        w.owns_dst=normalize?r.pos!=0:bool(r.owns_dst);
        w.owns_landing=normalize?r.src>=0 && r.dst>=0:bool(r.owns_landing);
        if(w.type==Wire::WIRE_TILE_PIN) { f.name(w.from,fpga::CB_NODE_LOCAL,w.local); }
        route.push_back(w);
      }
      for(size_t i=0;i<route.size();++i) {
        const auto &w=route[i];
        if(w.type!=Wire::WIRE_CROSSBAR) continue;
        Fixture::Node previous{w.from.x,w.from.y,w.pos?fpga::CB_NODE_DST:fpga::CB_NODE_LOCAL,w.local};
        auto add=[&](Fixture::Node next) { f.arc(previous,next); previous=next; };
        for(int j:{w.joint2,w.joint}) if(j>=0) add({w.from.x,w.from.y,fpga::CB_NODE_JOINT,j});
        if(w.jump>=0) {
          add({w.from.x,w.from.y,fpga::CB_NODE_SRC,w.jump});
          add({w.to.x,w.to.y,fpga::CB_NODE_DST,w.dst});
        } else if(i+1<route.size() && route[i+1].type==Wire::WIRE_TILE_PIN) {
          const auto &pin=route[i+1]; add({pin.from.x,pin.from.y,fpga::CB_NODE_LOCAL,pin.local});
        }
      }
      return f.install(id,std::move(route));
    };
    auto &requester=import(9600,{
      {1,{71,2},{71,2},145,0,-1,-1,-1,-1,-1,-1,-1,-1,1,0},
      {0,{71,2},{71,4},145,0,64,64,-1,-1,-1,-1,-1,-1,1,1},
      {0,{71,4},{76,4},64,1,1024,1024,-1,-1,-1,-1,-1,-1,0,0},
      {0,{76,4},{81,3},1024,1,736,736,-1,-1,-1,-1,-1,-1,1,0},
      {0,{81,3},{91,3},736,1,1025,1028,-1,-1,-1,-1,-1,-1,1,0},
      {0,{91,3},{91,9},1028,1,96,96,-1,-1,-1,-1,-1,-1,1,0},
      {0,{91,9},{91,15},96,1,96,96,-1,-1,-1,-1,-1,-1,1,0},
      {0,{91,15},{91,16},96,1,16,19,-1,-1,-1,-1,-1,-1,1,0},
      {0,{91,16},{91,18},19,1,64,64,-1,-1,-1,-1,-1,-1,1,0},
      {0,{91,18},{81,18},64,1,3073,3078,-1,-1,-1,-1,-1,-1,1,0},
      {0,{81,18},{70,18},3078,1,3073,3078,-1,-1,-1,-1,-1,-1,1,0},
      {0,{70,18},{67,17},3078,1,3809,3809,-1,-1,-1,-1,-1,-1,1,0},
      {0,{67,17},{67,18},3809,1,20,24,-1,-1,-1,-1,-1,-1,1,1},
      {0,{67,18},{67,20},24,1,65,65,-1,-1,-1,-1,-1,-1,1,1},
      {0,{67,20},{67,22},65,1,65,65,-1,-1,-1,-1,-1,-1,1,1},
      {0,{67,22},{67,23},65,1,21,25,-1,-1,-1,-1,-1,-1,1,1},
      {0,{67,23},{67,24},25,1,18,21,-1,-1,-1,-1,-1,-1,1,1},
      {0,{67,24},{70,24},21,1,260,263,-1,-1,-1,-1,-1,-1,1,1},
      {0,{70,24},{70,23},263,1,244,248,-1,-1,-1,-1,-1,-1,1,1},
      {0,{70,23},{67,22},248,1,3811,3811,-1,-1,-1,-1,-1,-1,1,1},
      {0,{67,22},{63,27},3811,1,3378,3378,-1,-1,-1,-1,-1,-1,1,1},
      {0,{63,27},{58,22},3378,1,3539,3539,-1,-1,-1,-1,-1,-1,1,1},
      {0,{58,22},{54,27},3539,1,3378,3378,-1,-1,-1,-1,-1,-1,1,1},
      {0,{54,27},{53,27},3378,1,3845,3841,-1,-1,-1,-1,-1,-1,1,1},
      {0,{53,27},{50,27},3841,1,3844,3840,-1,-1,-1,-1,-1,-1,1,1},
      {0,{50,27},{49,25},3840,1,3809,3809,-1,-1,-1,-1,-1,-1,1,1},
      {0,{49,25},{44,25},3809,1,3074,3072,-1,-1,-1,-1,-1,-1,1,1},
      {0,{44,25},{41,27},3072,1,3616,3616,-1,-1,-1,-1,-1,-1,0,0},
      {0,{41,27},{40,25},3616,1,3809,3809,-1,-1,-1,-1,-1,-1,1,0},
      {0,{40,25},{37,25},3809,1,3842,3847,-1,-1,-1,-1,-1,-1,1,0},
      {0,{37,25},{36,25},3847,1,3843,3848,-1,-1,-1,-1,-1,-1,1,1},
      {0,{36,25},{32,24},3848,1,3840,3845,-1,-1,-1,-1,-1,-1,1,1},
      {0,{32,24},{32,24},3845,1,245,240,-1,-1,-1,-1,-1,-1,1,1},
      {0,{32,24},{31,23},240,1,3840,3845,-1,-1,-1,-1,-1,-1,1,1},
      {0,{31,23},{31,23},3845,1,245,240,-1,-1,-1,-1,-1,-1,1,1},
      {0,{31,23},{26,22},240,1,3840,3845,-1,-1,-1,-1,-1,-1,1,1},
      {0,{26,22},{26,22},3845,1,245,240,-1,-1,-1,-1,-1,-1,1,1},
      {0,{26,22},{25,21},240,1,3840,3845,-1,-1,-1,-1,-1,-1,1,1},
      {0,{25,21},{25,21},3845,1,245,240,-1,-1,-1,-1,-1,-1,1,1},
      {3,{25,21},{25,19},-1,-1,-1,-1,-1,-1,4,240,4,247,1,0},
      {0,{25,19},{25,18},247,1,243,247,-1,-1,-1,-1,-1,-1,1,1},
      {0,{25,18},{25,17},247,1,243,247,-1,-1,-1,-1,-1,-1,1,1},
      {0,{25,17},{22,16},247,1,3810,3810,-1,-1,-1,-1,-1,-1,1,1},
      {0,{22,16},{21,15},3810,1,3810,3810,-1,-1,-1,-1,-1,-1,1,1},
      {0,{21,15},{21,13},3810,1,194,194,-1,-1,-1,-1,-1,-1,1,1},
      {0,{21,13},{15,9},194,1,3538,3538,-1,-1,-1,-1,-1,-1,1,1},
      {3,{15,9},{21,6},-1,-1,-1,-1,-1,-1,4,3538,4,21,1,0},
      {0,{21,6},{16,6},21,1,3845,3841,-1,-1,-1,-1,-1,-1,1,1},
      {0,{16,6},{16,6},3841,1,-1,-1,-1,-1,-1,-1,-1,-1,1,0},
      {1,{16,6},{16,6},91,3,-1,-1,-1,-1,-1,-1,-1,-1,1,0},
    },false);
    import(9602,{
      {1,{37,24},{37,24},148,12,-1,-1,-1,-1,-1,-1,-1,-1,1,0},
      {0,{37,24},{32,20},148,0,3539,3539,-1,-1,-1,-1,-1,-1,1,0},
      {0,{32,20},{31,19},3539,1,3811,3811,-1,-1,-1,-1,-1,-1,1,0},
      {0,{31,19},{25,15},3811,1,3539,3539,-1,-1,-1,-1,-1,-1,1,0},
      {0,{25,15},{26,14},3539,1,739,739,-1,-1,-1,-1,-1,-1,1,0},
      {0,{26,14},{31,15},739,1,547,547,-1,-1,-1,-1,-1,-1,1,0},
      {0,{31,15},{26,15},547,1,3846,3842,-1,-1,-1,-1,-1,-1,1,0},
      {0,{26,15},{26,14},3842,1,249,244,-1,-1,-1,-1,-1,-1,1,0},
      {0,{26,14},{25,14},244,1,3843,3848,-1,-1,-1,-1,-1,-1,1,0},
      {0,{25,14},{21,14},3848,1,3077,3074,-1,-1,-1,-1,-1,-1,1,1},
      {3,{21,14},{21,6},-1,-1,-1,-1,-1,-1,4,3074,4,3539,1,0},
      {0,{21,6},{21,1},3539,1,163,96,-1,-1,-1,-1,-1,-1,1,1},
      {0,{21,1},{21,7},96,1,96,96,-1,-1,-1,-1,-1,-1,1,1},
      {0,{21,7},{22,8},96,1,544,544,-1,-1,-1,-1,-1,-1,1,1},
      {0,{22,8},{22,7},544,1,241,245,-1,-1,-1,-1,-1,-1,1,1},
      {0,{22,7},{22,6},245,1,241,245,-1,-1,-1,-1,-1,-1,1,1},
      {0,{22,6},{22,4},245,1,192,192,-1,-1,-1,-1,-1,-1,1,1},
      {0,{22,4},{22,4},192,1,-1,-1,-1,-1,-1,-1,-1,-1,1,0},
      {1,{22,4},{22,4},81,3,-1,-1,-1,-1,-1,-1,-1,-1,1,0},
    },true);
    import(9601,{
      {1,{57,4},{57,4},155,11,-1,-1,-1,-1,-1,-1,-1,-1,1,0},
      {0,{57,4},{57,3},155,0,162,97,-1,-1,-1,-1,-1,-1,1,1},
      {0,{57,3},{49,3},97,1,3078,3079,-1,-1,-1,-1,-1,-1,1,1},
      {0,{49,3},{44,3},3079,1,3076,3073,-1,-1,-1,-1,-1,-1,1,1},
      {0,{44,3},{41,3},3073,1,3844,3840,-1,-1,-1,-1,-1,-1,1,1},
      {0,{41,3},{40,4},3840,1,246,3843,-1,-1,-1,-1,-1,-1,1,1},
      {0,{40,4},{37,4},3843,1,3846,3842,-1,-1,-1,-1,-1,-1,1,1},
      {0,{37,4},{37,5},3842,1,22,26,-1,-1,-1,-1,-1,-1,1,1},
      {0,{37,5},{37,7},26,1,67,67,-1,-1,-1,-1,-1,-1,1,1},
      {0,{37,7},{41,7},67,1,1030,1027,-1,-1,-1,-1,-1,-1,1,1},
      {0,{41,7},{44,6},1027,1,739,739,-1,-1,-1,-1,-1,-1,1,1},
      {0,{44,6},{49,2},739,2,979,979,-1,-1,-1,-1,-1,-1,0,0},
      {0,{49,2},{57,2},979,1,1031,1031,-1,-1,-1,-1,-1,-1,1,0},
      {0,{57,2},{58,2},1031,1,261,258,-1,-1,-1,-1,-1,-1,1,0},
      {0,{58,2},{62,2},258,1,259,257,-1,-1,-1,-1,-1,-1,1,0},
      {0,{62,2},{63,2},257,1,256,256,-1,-1,-1,-1,-1,-1,1,0},
      {0,{63,2},{66,3},256,1,544,544,-1,-1,-1,-1,-1,-1,1,1},
      {3,{66,3},{71,8},-1,-1,-1,-1,-1,-1,4,544,4,816,1,0},
      {0,{71,8},{71,2},816,1,160,160,-1,-1,-1,-1,-1,-1,1,1},
      {0,{71,2},{76,6},160,1,816,816,-1,-1,-1,-1,-1,-1,1,1},
      {0,{76,6},{75,7},816,1,246,3843,-1,-1,-1,-1,-1,-1,1,1},
      {0,{75,7},{75,7},3843,1,-1,-1,-1,-1,-1,-1,-1,-1,1,0},
      {1,{75,7},{75,7},87,3,-1,-1,-1,-1,-1,-1,-1,-1,1,0},
    },true);
    import(9601,{
      {1,{57,4},{57,4},155,11,-1,-1,-1,-1,-1,-1,-1,-1,1,0},
      {0,{57,4},{57,3},155,0,162,97,-1,-1,-1,-1,-1,-1,1,1},
      {0,{57,3},{49,3},97,1,3078,3079,-1,-1,-1,-1,-1,-1,1,1},
      {0,{49,3},{44,3},3079,1,3076,3073,-1,-1,-1,-1,-1,-1,1,1},
      {0,{44,3},{41,3},3073,1,3844,3840,-1,-1,-1,-1,-1,-1,1,1},
      {0,{41,3},{40,4},3840,1,246,3843,-1,-1,-1,-1,-1,-1,1,1},
      {0,{40,4},{37,4},3843,1,3846,3842,-1,-1,-1,-1,-1,-1,1,1},
      {0,{37,4},{37,5},3842,1,22,26,-1,-1,-1,-1,-1,-1,1,1},
      {0,{37,5},{37,7},26,1,67,67,-1,-1,-1,-1,-1,-1,1,1},
      {0,{37,7},{41,7},67,1,1030,1027,-1,-1,-1,-1,-1,-1,1,1},
      {0,{41,7},{44,6},1027,1,739,739,-1,-1,-1,-1,-1,-1,1,1},
      {0,{44,6},{49,2},739,2,979,979,-1,-1,-1,-1,-1,-1,0,0},
      {0,{49,2},{57,2},979,1,1031,1031,-1,-1,-1,-1,-1,-1,1,0},
      {0,{57,2},{58,2},1031,1,261,258,-1,-1,-1,-1,-1,-1,1,0},
      {0,{58,2},{62,2},258,1,259,257,-1,-1,-1,-1,-1,-1,1,0},
      {0,{62,2},{63,2},257,1,256,256,-1,-1,-1,-1,-1,-1,1,0},
      {0,{63,2},{66,3},256,1,544,544,-1,-1,-1,-1,-1,-1,1,1},
      {3,{66,3},{71,8},-1,-1,-1,-1,-1,-1,4,544,4,816,1,0},
      {0,{71,8},{71,2},816,1,160,160,-1,-1,-1,-1,-1,-1,1,1},
      {0,{71,2},{76,6},160,1,816,816,-1,-1,-1,-1,-1,-1,1,1},
      {0,{76,6},{75,7},816,1,246,3843,-1,-1,-1,-1,-1,-1,1,1},
      {0,{75,7},{75,7},3843,1,-1,-1,-1,-1,-1,-1,-1,-1,1,0},
      {1,{75,7},{75,7},95,7,-1,-1,-1,-1,-1,-1,-1,-1,1,0},
    },true);
    const std::vector<Fixture::Node> bypass{
      {71,2,0,145},
      {71,2,3,816},
      {76,6,4,816},
      {76,6,3,1025},
      {86,6,4,1028},
      {86,6,3,96},
      {86,12,4,96},
      {86,12,3,816},
      {92,16,4,816},
      {92,16,3,96},
      {92,22,4,96},
      {92,22,3,96},
      {92,29,4,96},
      {92,29,3,96},
      {92,35,4,96},
      {92,35,3,96},
      {92,41,4,96},
      {92,41,3,3376},
      {86,45,4,3376},
      {86,45,3,3073},
      {76,45,4,3078},
      {76,45,3,3376},
      {71,49,4,3376},
      {71,49,0,165},
      {71,49,3,3072},
      {63,49,4,3077},
      {63,49,3,3536},
      {58,45,4,3536},
      {58,45,3,160},
      {58,39,4,160},
      {58,39,3,160},
      {58,33,4,160},
      {58,33,3,3536},
      {54,29,4,3536},
      {54,29,3,3536},
      {50,24,4,3536},
      {50,24,3,3536},
      {45,20,4,3536},
      {45,20,3,3808},
      {44,19,4,3808},
      {44,19,3,976},
      {49,15,4,976},
      {49,15,3,241},
      {49,14,4,245},
      {49,14,0,178},
      {49,14,3,3072},
      {40,14,4,3077},
      {40,14,3,3072},
      {31,14,4,3077},
      {31,14,0,178},
      {31,14,0,128},
      {31,14,3,3539},
      {25,10,4,3539},
      {25,10,3,3539},
      {21,6,4,3539},
      {21,6,3,3845},
      {16,6,4,3841},
      {16,6,0,91},
    };
    for(size_t i=1;i<bypass.size();++i) f.arc(bypass[i-1],bypass[i]);
    f.rebuild(); f.device.rebuildIncomingDstMasks();
    for(auto &[id,s]:f.signals) {
      s->driver.outline.fixed=true; s->driver.pnr_completed=true;
      s->driver.coord=s->sinks.front().wires.front().front().from;
      f.tile(s->driver.coord).pin_state.leased_nodes.clearBit(s->sinks.front().wires.front().front().local);
      s->net.pnr_preserved=true;
      for(auto &sink:s->sinks) {sink.outline.fixed=true;sink.pnr_completed=true;sink.coord=sink.wires.front().back().from;}
    }
    // Without the Takeoff owner, the same entry must work from an exhausted
    // free frontier too, not only inside the combined Takeoff/Grounding walk.
    if(mode==1) require(fpga::unrouteNet(f.signal(9601).net),"cannot remove Takeoff control owner");
    if(mode==2) f.signal(9602).net.route_protected=true;
    const auto before=f.state();
    auto &s=f.signal(9600);const auto binding=s.net.routes[0];
    std::ostringstream free_proof;
    require(!fpga::auditRouteCompletion(s.net,binding,requester.front(),requester.back(),
        free_proof,{},8192,8192,200),"snapshot unexpectedly has a free route");
    require(f.state()==before,"free-only proof changed recorded occupancy");
    // Independently prove all busy resources on the complete witness belong
    // to the three genuine transit victims, and that their cuts can release
    // them. An optimistic reachability result alone is insufficient.
    std::vector<fpga::RouteCutNode> cuts{{{21,6},fpga::CB_NODE_DST,3539}};
    if(mode!=1) { cuts.push_back({{71,2},fpga::CB_NODE_SRC,816});cuts.push_back({{76,6},fpga::CB_NODE_DST,816}); }
    const auto victims=fpga::findNetRouteCutVictims(cuts);
    require(victims.size()==(mode==1?1:3),"snapshot selected unexpected transit dependencies");
    if(mode!=2) for(const auto &v:victims)
      require(fpga::canPreemptNetRouteFromNodes(*v.net,v.binding_index,cuts),"witness cuts protected endpoint resources");
    pnr::RouteDesign router;router.fpga=&f.device;router.moving_stage=true;
    router.route_preemption_enabled=mode!=3;
    pnr::RouteDesign::RouteTask task{.from=&s.driver,.to=&s.sinks[0],.net=&s.net,
        .from_port="out",.to_port="in",.net_name=binding.route_name};
    const bool done=router.repairTaskGaps(task,0,8192,false);
    std::printf("WINDOW_ENTRY mode=%d done=%d cuts=%zu queued=%zu\n",mode,done,
                size_t(router.route_stats.preempt_success),router.pending_route_todo.size());
    require(!router.moving_focus_inst,"wire repair leaked its temporary focus");
    require(s.driver.outline.fixed && s.sinks[0].outline.fixed &&
            same(s.driver.coord,{71,2}) && same(s.sinks[0].coord,{16,6}),"wire repair moved fixed cells");
    const bool expected=mode<2 && !reproduce;
    require(done==expected,expected?"Grounding ignored a free SRC entering its window":"blocked window entry committed an invalid route");
    if(done) {
      require(fpga::isRouteComplete(requester),"window entry left requester partial");
      int hops=0;
      for(const auto &w:requester)
        if((w.type==Wire::WIRE_CROSSBAR && w.jump>=0) || w.type==Wire::WIRE_ROUTE_EDGE)
          hops+=!same(w.from,w.to);
      std::printf("WINDOW_ENTRY mode=%d complete_hops=%d witness_hops=26\n",mode,hops);
      // Imported owner topology also exposes alternative legitimate paths.
      // Validate the chosen path's physical continuity and normal hop limit,
      // rather than requiring it to reproduce the optimistic BFS's ordering.
      require(hops>0 && hops<=200,"window entry violated the caller's hop limit");
      check_complete_path(f,requester,hops);f.state();
    }
    else require(f.state()==before && router.pending_route_todo.empty(),"failed window entry damaged transit owners");
  }
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

// Example 1's destination recovery alternated these two real paths hundreds
// of times: one repair took (49,21)/SRC978 and its (53,17)/DST978 landing,
// then the displaced peer took them back. Occupancy was correct: both had
// registered owners. The failure snapshot also proved a FREE 140-node bypass
// through arbitrary LOCAL bridges. Import that bypass and both retained paths
// with neutral names. A broad, dead free frontier reproduces budget pressure;
// checking only the first completed repair would hide the repeated exchange.
void moving_gap_free_before_cut_regression() {
  for(bool preemption : {true,false}) {
  Fixture f(104,80);
  const std::vector<Fixture::Node> free_path{
      {36,36,0,143},
      {36,36,3,3843},
      {32,36,4,3848},
      {32,36,0,184},
      {32,36,3,98},
      {32,42,4,98},
      {32,42,3,98},
      {32,48,4,98},
      {32,48,3,98},
      {32,55,4,98},
      {32,55,3,1029},
      {54,55,4,1030},
      {54,55,3,98},
      {54,61,4,98},
      {54,61,3,98},
      {54,67,4,98},
      {54,67,3,98},
      {54,73,4,98},
      {54,73,3,98},
      {54,80,4,98},
      {54,80,3,98},
      {54,86,4,98},
      {54,86,3,98},
      {54,92,4,98},
      {54,92,3,818},
      {58,96,4,818},
      {58,96,3,818},
      {63,100,4,818},
      {63,100,3,1029},
      {71,100,4,1030},
      {71,100,3,1028},
      {76,100,4,1026},
      {76,100,0,108},
      {77,100,0,76},
      {78,103,0,179},
      {78,103,0,54},
      {78,103,0,44},
      {78,103,0,92},
      {78,99,0,60},
      {78,98,0,60},
      {78,97,0,60},
      {78,96,0,60},
      {78,95,0,60},
      {78,94,0,60},
      {78,93,0,60},
      {78,92,0,60},
      {78,91,0,60},
      {78,91,0,95},
      {78,89,0,60},
      {78,88,0,60},
      {78,87,0,60},
      {78,86,0,60},
      {78,85,0,60},
      {78,84,0,60},
      {78,83,0,60},
      {78,78,0,416},
      {78,73,0,60},
      {78,72,0,60},
      {78,71,0,60},
      {78,70,0,60},
      {78,69,0,60},
      {78,68,0,60},
      {78,67,0,60},
      {78,66,0,60},
      {78,66,0,95},
      {78,64,0,60},
      {78,63,0,60},
      {78,62,0,60},
      {78,61,0,60},
      {78,60,0,60},
      {78,59,0,60},
      {78,58,0,60},
      {78,57,0,60},
      {78,56,0,60},
      {78,55,0,60},
      {78,54,0,60},
      {78,53,0,60},
      {78,52,0,28},
      {78,51,0,60},
      {78,50,0,60},
      {78,49,0,60},
      {78,48,0,60},
      {78,47,0,60},
      {78,46,0,60},
      {78,45,0,60},
      {78,44,0,60},
      {78,43,0,60},
      {78,42,0,60},
      {78,41,0,60},
      {78,40,0,60},
      {78,39,0,60},
      {78,39,0,95},
      {78,37,0,60},
      {78,36,0,60},
      {78,35,0,60},
      {78,34,0,60},
      {78,33,0,60},
      {78,32,0,60},
      {78,31,0,60},
      {78,26,0,416},
      {78,26,0,341},
      {78,26,0,277},
      {78,26,0,117},
      {77,26,0,5},
      {76,26,0,5},
      {75,26,0,41},
      {74,26,0,5},
      {73,26,0,5},
      {72,26,0,5},
      {71,26,0,5},
      {70,26,0,41},
      {69,26,0,5},
      {68,26,0,5},
      {67,26,0,5},
      {66,26,0,41},
      {65,26,0,5},
      {64,26,0,5},
      {63,26,0,5},
      {62,26,0,41},
      {61,26,0,5},
      {60,26,0,5},
      {59,26,0,5},
      {58,26,0,5},
      {57,26,0,41},
      {56,26,0,5},
      {55,26,0,5},
      {54,26,0,5},
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
      {54,16,0,77},
      {54,16,0,83},
  };
  for(size_t i=1;i<free_path.size();++i) f.arc(free_path[i-1],free_path[i]);
  const std::vector<Fixture::Node> short_path{
      {36,36,0,143},
      {36,36,3,3843},
      {32,36,4,3848},
      {32,36,3,3840},
      {31,35,4,3845},
      {31,35,0,196},
      {31,35,3,99},
      {31,41,4,99},
      {31,41,0,128},
      {31,41,3,3539},
      {25,37,4,3539},
      {25,37,3,739},
      {26,36,4,739},
      {26,36,0,128},
      {26,36,0,184},
      {26,36,3,978},
      {32,32,4,978},
      {32,32,3,738},
      {36,31,4,738},
      {36,31,3,259},
      {37,31,4,257},
      {37,31,3,737},
      {40,30,4,737},
      {40,30,3,161},
      {40,23,4,161},
      {40,23,3,247},
      {40,22,4,241},
      {40,22,3,245},
      {40,22,4,240},
      {40,22,3,244},
      {40,21,4,248},
      {40,21,0,165},
      {40,21,3,1029},
      {49,21,4,1030},
      {49,21,3,978},
      {53,17,4,978},
      {53,17,3,194},
      {53,15,4,194},
      {53,15,3,243},
      {53,14,4,247},
      {53,14,3,259},
      {54,14,4,257},
      {54,14,3,545},
      {57,15,4,545},
      {57,15,3,3617},
      {54,16,4,3617},
      {54,16,0,83},
  };
  for(size_t i=1;i<short_path.size();++i) f.arc(short_path[i-1],short_path[i]);
  auto graph=[&](Coord a,int ak,int ai,Coord b,int bk,int bi) {
    f.arc({a.x,a.y,ak,ai},{b.x,b.y,bk,bi});
    Wire w; w.type=Wire::WIRE_ROUTE_EDGE; w.from=a; w.to=b;
    w.from_node_type=ak; w.from_node=ai; w.to_node_type=bk; w.to_node=bi; return w;
  };
  auto gap=[](Coord a,int ak,int ai,Coord b,int bk,int bi) {
    Wire w; w.type=Wire::WIRE_GAP; w.from=a; w.to=b;
    w.from_node_type=ak; w.from_node=ai; w.to_node_type=bk; w.to_node=bi; return w;
  };
  auto ground=[](Coord at,int dst) { Wire w; w.from=w.to=at; w.local=dst; w.pos=1; return w; };
  Route route_901{
      f.pin({36,36},143,true),
      f.edge({36,36},{32,36},143,3843,3848,0),
      f.edge({32,36},{31,35},3848,3840,3845,1),
      graph({31,35},4,3845,{31,35},0,196),
      graph({31,35},0,196,{31,35},3,99),
      graph({31,35},3,99,{31,41},4,99),
      graph({31,41},4,99,{31,41},0,128),
      graph({31,41},0,128,{31,41},3,3539),
      graph({31,41},3,3539,{25,37},4,3539),
      f.edge({25,37},{26,36},3539,739,739,1),
      graph({26,36},4,739,{26,36},0,128),
      graph({26,36},0,128,{26,36},0,184),
      graph({26,36},0,184,{26,36},3,978),
      graph({26,36},3,978,{32,32},4,978),
      f.edge({32,32},{36,31},978,738,738,1),
      f.edge({36,31},{37,31},738,259,257,1),
      f.edge({37,31},{40,30},257,737,737,1),
      f.edge({40,30},{40,23},737,161,161,1),
      f.edge({40,23},{40,22},161,247,241,1),
      f.edge({40,22},{40,22},241,245,240,1),
      f.edge({40,22},{40,21},240,244,248,1),
      graph({40,21},4,248,{40,21},0,165),
      graph({40,21},0,165,{40,21},3,1029),
      graph({40,21},3,1029,{49,21},4,1030),
      gap({49,21},4,1030,{53,15},4,194),
      f.edge({53,15},{53,14},194,243,247,1),
      f.edge({53,14},{54,14},247,259,257,1),
      f.edge({54,14},{57,15},257,545,545,1),
      f.edge({57,15},{54,16},545,3617,3617,1),
      ground({54,16},3617),
      f.pin({54,16},83,false)};
  f.arc({54,16,4,3617},{54,16,0,83});
  Route route_902{
      f.pin({41,16},142,true),
      f.edge({41,16},{44,16},142,256,256,0),
      f.edge({44,16},{44,18},256,64,64,1),
      f.edge({44,18},{41,19},64,3616,3616,1),
      f.edge({41,19},{41,20},3616,20,24,1),
      f.edge({41,20},{41,22},24,65,65,1),
      f.edge({41,22},{40,22},65,3844,3840,1),
      f.edge({40,22},{37,23},3840,246,3843,1),
      f.edge({37,23},{32,23},3843,3079,3075,1),
      f.edge({32,23},{32,23},3075,23,23,1),
      f.edge({32,23},{32,25},23,64,64,1),
      f.edge({32,25},{36,27},64,544,544,1),
      f.edge({36,27},{36,25},544,241,245,1),
      graph({36,25},4,245,{36,25},0,178),
      graph({36,25},0,178,{36,25},3,816),
      graph({36,25},3,816,{40,30},4,816),
      f.edge({40,30},{41,31},816,240,259,1),
      f.edge({41,31},{45,31},259,1030,1027,1),
      f.edge({45,31},{45,29},1027,195,195,1),
      f.edge({45,29},{49,28},195,739,739,1),
      f.edge({49,28},{49,21},739,163,163,1),
      graph({49,21},4,163,{49,21},0,165),
      graph({49,21},0,165,{49,21},3,978),
      graph({49,21},3,978,{53,17},4,978),
      f.edge({53,17},{53,18},978,18,21,1),
      f.edge({53,18},{54,19},21,546,546,1),
      ground({54,19},546),
      f.pin({54,19},108,false)};
  f.arc({54,19,4,546},{54,19,0,108});
  // These are real registered leases, including the internal LOCAL bridges.
  auto install=[&](int id,Route route)->Route& {
    auto lease=[&](Coord at,int kind,int node) {
      auto &cb=f.tile(at).cb;
      switch(kind) {
        case fpga::CB_NODE_LOCAL:cb.local.local.setBit(node);break;
        case fpga::CB_NODE_JOINT:cb.joint.jump.setBit(node);break;
        case fpga::CB_NODE_SRC:cb.src.jump.setBit(node);break;
        case fpga::CB_NODE_DST:cb.dst.jump.setBit(node);break;
      }
    };
    for(auto &w:route) {
      if(w.type==Wire::WIRE_CROSSBAR && w.jump>=0) w.owns_landing=true;
      if(w.type==Wire::WIRE_ROUTE_EDGE) {
        lease(w.from,w.from_node_type,w.from_node);lease(w.to,w.to_node_type,w.to_node);
      }
    }
    auto &out=f.install(id,std::move(route));
    f.tile(out.front().from).pin_state.leased_nodes.clearBit(out.front().local);
    return out;
  };
  auto &interrupted=install(901,std::move(route_901));
  auto &peer=install(902,std::move(route_902));
  f.edge({49,21},{49,22},1030,1900,1900);
  for(int i=0;i<512;++i) {
    f.edge({49,22},{49,23},1900,2050+i,2050+i);
    f.arc({49,23,fpga::CB_NODE_DST,2050+i},{49,23,fpga::CB_NODE_LOCAL,3000+i});
  }
  f.rebuild(); f.device.rebuildIncomingDstMasks();
  const auto before=f.state();
  auto &s=f.signal(901);const auto binding=s.net.routes[0];
  std::ostringstream witness;
  require(fpga::auditRouteCompletion(s.net,binding,interrupted.front(),interrupted.back(),
      witness,{},4096,4096,200),"captured free 140-node bypass is not feasible");
  require(f.state()==before,"read-only bypass audit changed real owners");
  auto peer_history=[&] { std::ostringstream out;
    fpga::dumpNetRouteHistory(f.signal(902).net,out);return out.str(); };
  const auto peer_before=peer_history();
  pnr::RouteDesign router;router.fpga=&f.device;router.moving_stage=true;
  router.route_preemption_enabled=preemption;
  for(auto &[id,signal]:f.signals) {
    signal->driver.outline.fixed=true;
    for(auto &sink:signal->sinks) sink.outline.fixed=true;
  }
  pnr::RouteDesign::RouteTask task{.from=&s.driver,.to=&s.sinks[0],.net=&s.net,
      .from_port="out",.to_port="in",.net_name=binding.route_name};
  // Ordinary queued gap repair, NOT the source_probe=true shortcut covered
  // by the older LOCAL-cycle test. Use the existing 200-hop limit throughout.
  const bool done=router.repairTaskGaps(task,0,256,false);
  std::cout << "Queued gap free-first: preemption=" << preemption << " complete=" << done
      << " cuts=" << router.route_stats.preempt_success
      << " peer_complete=" << fpga::isRouteComplete(peer) << '\n';
  require(done && fpga::isRouteComplete(interrupted),"queued repair missed the free bypass");
  require(fpga::isRouteComplete(peer) && router.route_stats.preempt_success==0 &&
      router.pending_route_todo.empty(),"queued repair evicted its peer despite a free bypass");
  require(peer_history()==peer_before,"free repair changed its peer geometry or ownership");
  require(s.driver.tile.peer==f.device.getTile(36,36) &&
      s.sinks[0].tile.peer==f.device.getTile(54,16),"wire repair moved a fixed endpoint");
  require(!router.moving_focus_inst,"queued repair did not restore caller focus");
  f.state();
  }
}

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

// A blocked terminal cannot be repaired by flooding an unrelated free area.
// Keep free and same-signal terminal entries as reverse controls: these must
// still use the ordinary free search, with no unnecessary transit eviction.
void blocked_terminal_gap_regression() {
  for (int joints : {0, 1, 2}) for (int mode : {0, 1, 2, 3}) {
    Fixture f(5, 6);
    auto first = f.edge({1,1},{2,1},144,200,500,0);
    first.owns_landing = true;
    Wire gap;
    gap.type = Wire::WIRE_GAP;
    gap.from = {2,1}; gap.from_node_type = fpga::CB_NODE_DST; gap.from_node = 500;
    gap.to = {4,1}; gap.to_node_type = fpga::CB_NODE_LOCAL; gap.to_node = 91;
    auto &route = f.install(811, {f.pin({1,1},144,true), first, gap, f.pin({4,1},91,false)});
    f.tile({1,1}).pin_state.leased_nodes.clearBit(144);
    auto middle = f.edge({2,1},{3,1},500,200,500);
    auto last = f.edge({3,1},{4,1},500,200,500);
    Route terminal;
    f.terminal(terminal,{4,1},500,91,joints ? 31 : -1,joints == 2 ? 32 : -1);
    if (mode == 2) {
      // A sibling owns the exact approach, but not the target local. It must
      // remain reusable by this signal instead of being treated as foreign.
      Route sibling{f.pin({1,1},144,true),first,middle,last,
          f.edge({4,1},{4,0},500,3990,3990)};
      f.terminal(sibling,{4,0},3990,3990);
      f.install(811,std::move(sibling));
      for (size_t i=0;i<2;++i) route[i].shared=true;
    } else {
      Route victim{f.pin({4,3},3990,true),
          f.edge({4,3},{4,2},3990,3990,3990,0),
          f.edge({4,2},{4,1},3990,3990,500),
          f.edge({4,1},{4,0},500,3990,3990)};
      f.terminal(victim,{4,0},3990,3990);
      f.install(812,std::move(victim));
      f.tile({4,3}).pin_state.leased_nodes.clearBit(3990);
    }
    if (mode == 1) {
      // One free alternative is sufficient to forbid the blocked-terminal
      // shortcut, even when every other terminal entry is occupied.
      f.edge({3,1},{4,1},500,201,501);
      Route alternative;
      f.terminal(alternative,{4,1},501,91,joints ? 31 : -1,joints == 2 ? 32 : -1);
    }
    for (int n=0;n<256;++n) {
      f.edge({2,1},{2,2},500,1000+n,1000+n);
      f.arc({2,2,fpga::CB_NODE_DST,1000+n},{2,2,fpga::CB_NODE_LOCAL,2000+n});
    }
    f.rebuild(); f.device.rebuildIncomingDstMasks();
    const auto before = f.state(); // Every busy bit must have an installed owner.
    int commits = 0;
    auto preempt = [&](const std::vector<fpga::RouteCutNode>& cuts, bool commit,
                       std::vector<fpga::RouteCutNode>& released) {
      if (mode != 0 || cuts.empty()) return false;
      for (const auto& cut : cuts)
        if (!same(cut.tile,{4,1}) || cut.type != fpga::CB_NODE_DST || cut.node != 500)
          return false;
      auto victims = fpga::findNetRouteCutVictims(cuts);
      released = fpga::previewNetRouteCuts(victims,cuts,true);
      if (!commit) return !released.empty();
      ++commits;
      return fpga::unrouteNetRoutesFromNodes(victims,cuts,true);
    };
    struct Trace : fpga::RouteSearchTrace {
      std::ostringstream out;
      std::ostream& stream() override { return out; }
      void snapshot(Tile&) override {}
    } trace;
    auto& net=f.signal(811).net;
    // Cancellation must not preempt anything, release retained fragments, or
    // reinterpret the caller's search budget as permission to change leases.
    require(!fpga::repairNetRouteGaps(net,0,[]{return true;},4096,200,{},preempt) &&
        f.state()==before && commits==0,"cancelled blocked-terminal probe changed state");
    const bool done=fpga::repairNetRouteGaps(net,0,{},4096,200,{},preempt);
    require(done == (mode != 3) && fpga::isRouteComplete(route) == done,
        "blocked-terminal repair ignored its allowed completion/cut policy");
    const bool expanded = trace.out.str().find("phase=gap_pop") != std::string::npos;
    std::cout << "Blocked terminal joints=" << joints << " mode=" << mode
        << " gap_expanded=" << expanded << " cuts=" << commits << '\n';
    // No free exact-gap completion exists in mode zero. Do not explore its
    // 256 irrelevant exits before invoking the existing Grounding fallback.
    require(expanded == (mode == 1 || mode == 2),"blocked terminal flooded impossible free-only gap search");
    require(commits == int(mode == 0),"free/shared terminal unnecessarily preempted a route");
    if (done) check_complete_path(f,route,3);
    // Refusing the only possible cut must leave all retained/victim leases
    // intact; proving the cut is not permission to evict its owner.
    else require(f.state()==before,"refused terminal cut changed routes or leases");
    f.state(); // Check repaired and victim ownership after the committed cut.
  }
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
    // Keep a physically incoming free terminal entry, unreachable from this
    // source. This test exercises frontier reuse, not the proven-cut shortcut.
    f.edge({length+1,2},{length,1},700,700,701,0);
    f.arc({length,1,fpga::CB_NODE_DST,701},{length,1,fpga::CB_NODE_LOCAL,91});
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
    struct Cancel { int polls=0; bool operator()() { ++polls; return true; } };
    std::function<bool()> cancel=Cancel{};
    require(!fpga::repairNetRouteGaps(net,0,cancel,4096,200,{},preempt) &&
        f.state() == before, "cancelled GAP probe changed routing state");
    // Profiling must call the original stateful callback, not a copied target.
    require(cancel.target<Cancel>()->polls==1,"profiling copied or repolled cancellation");
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

// A production-sized neutral fabric: a deep stem opens into 65 layers of
// 4096 lanes. Converging exits amplify per-candidate ancestry reconstruction.
static void gapCostFabric(Fixture &f, int fanout) {
  for (int x = 1; x < 33; ++x)
    f.edge({x, 1}, {x + 1, 1}, 500, 200, 500);
  for (int lane = 0; lane < 4096; ++lane)
    f.edge({33, 1}, {34, 1}, 500, lane, lane);
  for (int x = 34; x < 99; ++x) {
    auto &cb = *f.tile({x, 1}).cb_type;
    for (int lane = 0; lane < 4096; ++lane) {
      f.edge({x, 1}, {x + 1, 1}, lane, lane, lane);
      for (int offset = 1; offset < fanout; ++offset)
        cb.dst_src[lane].jump.setBit((lane + offset) % 4096);
    }
  }
}

struct GapCostProfile {
  std::map<std::string, std::string> old;
  GapCostProfile() {
    for (const char *key : {"SCALEPNR_PROFILE_REPAIR", "SCALEPNR_FREE_SEARCH_STATES"})
      if (const char *value = std::getenv(key))
        old[key] = value;
    setenv("SCALEPNR_PROFILE_REPAIR", "1", 1);
    setenv("SCALEPNR_FREE_SEARCH_STATES", "64", 1);
    fpga::resetRepairStatistics();
  }
  ~GapCostProfile() {
    for (const char *key : {"SCALEPNR_PROFILE_REPAIR", "SCALEPNR_FREE_SEARCH_STATES"}) {
      auto it = old.find(key);
      if (it == old.end())
        unsetenv(key);
      else
        setenv(key, it->second.c_str(), 1);
    }
  }
};

static Wire gapCostHole() {
  Wire gap;
  gap.type = Wire::WIRE_GAP;
  gap.from = {1, 1};
  gap.from_node_type = fpga::CB_NODE_DST;
  gap.from_node = 500;
  gap.to = {3, 3};
  gap.to_node_type = fpga::CB_NODE_DST;
  gap.to_node = 700;
  return gap;
}

// Reproduce the native billion-scale ancestry work without wall-clock limits.
// Failure is read-only: all busy nodes have real installed route owners.
// The next real deadline was reached after source recovery, rather than in
// private input proofs. Its numeric Takeoff search revisited cheap labels
// only after walking each candidate's complete ancestry. Reproduce that
// convergent search behind a genuinely transit-owned first exit. Every node
// name is generated from a neutral pattern; no bare busy bits are installed.
void source_takeoff_work_regression(bool connect_sink) {
  GapCostProfile profile;
  // Match the real run's default. Its explicit caller budget is the free
  // budget too; the optional fast-cap escalation is a different scenario.
  unsetenv("SCALEPNR_FREE_SEARCH_STATES");
  Fixture f(6,32);
  f.node_pattern="probe_lane";
  constexpr int lanes=128, last=25;
  f.edge({0,1},{1,1},144,200,500,0);
  for(int lane=0;lane<lanes;++lane)
    f.edge({1,1},{2,1},500,1000+lane,1000+lane);
  for(int x=2;x<last;++x) for(int lane=0;lane<lanes;++lane) {
    f.edge({x,1},{x+1,1},1000+lane,1000+lane,1000+lane);
    for(int offset=1;offset<32;++offset)
      f.tile({x,1}).cb_type->dst_src[1000+lane].jump.setBit(1000+(lane+offset)%lanes);
  }
  Route victim{f.pin({0,4},3990,true),f.edge({0,4},{0,3},3990,3990,3990,0),
      f.edge({0,3},{0,1},3990,3990,950),f.edge({0,1},{1,1},950,200,500),
      f.edge({1,1},{1,3},500,3990,3990)};
  f.terminal(victim,{1,3},3990,3990);
  f.install(851,std::move(victim));
  Route terminal;
  f.terminal(terminal,{30,4},900,91);
  // A real free grounding exists, but its predecessor has no source path.
  f.edge({29,4},{30,4},801,801,900);
  if(connect_sink)f.edge({last,1},{30,4},1000,3900,900);
  auto& signal=f.signal(852);
  signal.driver.tile.set(&f.device.tile_grid[32]);
  signal.sinks.emplace_back();signal.sink_cells.emplace_back();
  auto& sink=signal.sinks.back();
  signal.sink_cells.back().name="probe_receiver";
  signal.sink_cells.back().type="logic_731";
  sink.cell_ref.set(&signal.sink_cells.back());
  sink.tile.set(&f.device.tile_grid[4*32+30]);
  rtl::NetRouteBinding binding;
  binding.from=&signal.driver;binding.to=&sink;
  binding.from_port="out";binding.to_port="in";binding.route_name="probe_request";
  const auto from=f.pin({0,1},144,true),to=f.pin({30,4},91,false);
  f.rebuild();f.device.rebuildIncomingDstMasks();
  const auto before=f.state();
  int previews=0,commits=0;
  bool allow_cut=true;
  auto preempt=[&](const std::vector<fpga::RouteCutNode>& cuts,bool commit,
                   std::vector<fpga::RouteCutNode>& released) {
    if(!allow_cut)return false;
    if(cuts.empty())return false;
    for(const auto& cut:cuts)
      if(!((same(cut.tile,{0,1}) && cut.type==fpga::CB_NODE_SRC && cut.node==200) ||
           (same(cut.tile,{1,1}) && cut.type==fpga::CB_NODE_DST && cut.node==500)))return false;
    const auto victims=fpga::findNetRouteCutVictims(cuts);
    if(victims.empty())return false;
    for(const auto& v:victims)require(v.net==&f.signal(851).net,"probe selected a different owner");
    released=fpga::previewNetRouteCuts(victims,cuts,true);
    if(!commit){++previews;return !released.empty();}
    ++commits;return fpga::unrouteNetRoutesFromNodes(victims,cuts,true);
  };
  fpga::UnboundRouteSearch search(signal.net,binding);
  if(connect_sink) {
    // Probe reuse must stop at the public request boundary. A different
    // owner policy, a cancelled request, and a larger budget must all be
    // reconsidered, even on this same reusable search object.
    allow_cut=false;
    require(!search.route(from,to,{},8192,100,preempt) && f.state()==before && !commits,
        "declined owner policy changed the transit route");
    allow_cut=true;
    require(!search.route(from,to,[]{return true;},8192,100,preempt) && f.state()==before && !commits,
        "cancelled probe changed the transit route");
    bool limited=false;
    require(!search.route(from,to,{},64,100,preempt,{},&limited) && limited &&
        f.state()==before && !commits,
        "small-budget probe did not retain its owner or its limit flag");
    fpga::resetRepairStatistics();
  }
  const bool done=search.route(from,to,{},8192,100,preempt);
  require(done==connect_sink,"Takeoff work fixture lost its reachable/unreachable control");
  require(previews>=2,"fixture never explored its transit-owned first jump");
  if(done) {
    require(commits==1,"complete probe did not make one atomic cut");
    check_complete_path(f,sink.wires.front(),26);
    f.state();
  } else require(!commits && sink.wires.empty() && f.state()==before,
      "failed probe changed placement, published routing, or victim leases");
  using C=fpga::RepairCounter;
  const auto stats=fpga::repairStatistics();
  std::printf("Takeoff work: connected=%d expanded=%llu edges=%llu parents=%llu\n",connect_sink,
      (unsigned long long)stats[C::source_probe_expanded],
      (unsigned long long)stats[C::source_probe_edges],
      (unsigned long long)stats[C::source_probe_parent_steps]);
  // All 47 live layers plus source/exit/first landing must be visited ONCE.
  // The 128 final dead-end landings have no continuation and are ineligible.
  if(!connect_sink)require(stats[C::source_probe_expanded]==uint64_t((2*last-3)*lanes+3) &&
      stats[C::source_probe_edges]>stats[C::source_probe_expanded]*4,
      "Takeoff probe repeated or skipped the broad reachable graph");
  require(stats[C::source_probe_parent_steps]<=stats[C::source_probe_expanded]*70,
      "Takeoff completion walks ancestry again for already dominated candidates");
}

void gap_parent_work_regression() {
  GapCostProfile profile;
  {
    Fixture f(6, 102);
    auto first = f.edge({0, 1}, {1, 1}, 144, 200, 500, 0);
    first.owns_landing = true;
    Route r{f.pin({0, 1}, 144, true), first, gapCostHole()};
    f.terminal(r, {3, 3}, 700, 91);
    f.install(840, std::move(r));
    f.tile({0, 1}).pin_state.leased_nodes.clearBit(144);
    gapCostFabric(f, 32);
    f.rebuild();
    const auto before = f.state();
    require(!fpga::repairNetRouteGaps(f.signal(840).net, 0, {}, 262144, 200),
            "isolated gap endpoint unexpectedly reached");
    require(f.state() == before, "failed ancestry search changed retained leases");
    using C = fpga::RepairCounter;
    const auto stats = fpga::repairStatistics();
    fpga::printRepairStatistics(std::cout, "gap ancestry production scale");
    // Keep the production-sized exploration; speed must not come from pruning
    // reachable lanes, reducing the budget, or silently cancelling the search.
    require(stats[C::gap_expanded] == 262144 && stats[C::gap_edges] > 8000000,
            "ancestry test did not reproduce the broad convergent search");
    // At most one ancestry pass per expanded state, not one per SRC/landing.
    require(stats[C::gap_parent_steps] <= stats[C::gap_expanded] * 101,
            "gap search repeatedly walked ancestry per candidate exit");
  }
  {
    // The optimized scratch masks must still reject a joint and SRC used in
    // an earlier visit, while allowing a second visit through other resources.
    Fixture small(3, 5);
    auto takeoff = small.edge({0, 1}, {1, 1}, 144, 200, 500, 0);
    takeoff.owns_landing = true;
    auto hole = gapCostHole();
    hole.to = {3, 1};
    hole.to_node = 900;
    Route path{small.pin({0, 1}, 144, true), takeoff, hole};
    small.terminal(path, {3, 1}, 900, 91);
    auto &complete = small.install(843, std::move(path));
    small.tile({0, 1}).pin_state.leased_nodes.clearBit(144);
    small.edge({1, 1}, {2, 1}, 500, 20, 600);
    small.tile({1, 1}).cb_type->dst_src[500].jump.clearBit(20);
    small.arc({1, 1, fpga::CB_NODE_DST, 500}, {1, 1, fpga::CB_NODE_JOINT, 7});
    small.arc({1, 1, fpga::CB_NODE_JOINT, 7}, {1, 1, fpga::CB_NODE_SRC, 20});
    small.edge({2, 1}, {1, 1}, 600, 20, 501);
    small.arc({1, 1, fpga::CB_NODE_DST, 501}, {1, 1, fpga::CB_NODE_JOINT, 7});
    small.arc({1, 1, fpga::CB_NODE_DST, 501}, {1, 1, fpga::CB_NODE_SRC, 20});
    small.edge({1, 1}, {4, 1}, 501, 21, 901);
    small.tile({1, 1}).cb_type->dst_src[501].jump.clearBit(21);
    small.arc({1, 1, fpga::CB_NODE_JOINT, 7}, {1, 1, fpga::CB_NODE_SRC, 21});
    small.edge({1, 1}, {3, 1}, 501, 22, 900);
    small.tile({1, 1}).cb_type->dst_src[501].jump.clearBit(22);
    small.arc({1, 1, fpga::CB_NODE_DST, 501}, {1, 1, fpga::CB_NODE_JOINT, 8});
    small.arc({1, 1, fpga::CB_NODE_JOINT, 8}, {1, 1, fpga::CB_NODE_SRC, 22});
    small.rebuild();
    struct Trace : fpga::RouteSearchTrace {
      std::ostringstream out;
      std::ostream &stream() override { return out; }
      void snapshot(Tile &) override {}
    } trace;
    require(fpga::repairNetRouteGaps(small.signal(843).net, 0, {}, 64, 10),
            "ancestry scratch rejected the legal second visit");
    check_complete_path(small, complete, 4);
    require(std::any_of(
                complete.begin(), complete.end(),
                [](const Wire &w) { return same(w.from, {1, 1}) && w.jump == 22 && w.joint == 8; }),
            "ancestry scratch allowed the already-used joint instead of its free alternative");
    // Check the actual rejection reasons, not just the final successful path:
    // either lost scratch mask could otherwise be hidden by the seen-node set.
    const auto decision = "phase=gap_exit from=(1,1)/" + std::to_string(fpga::CB_NODE_DST) +
                          ":501 to=(1,1)/" + std::to_string(fpga::CB_NODE_SRC) + ":";
    require(trace.out.str().find(decision + "20 result=prefix_src") != std::string::npos &&
                trace.out.str().find(decision + "21 result=joint_blocked") != std::string::npos,
            "ancestry scratch lost a temporary SRC or joint from the earlier visit");
    small.state();
  }
}

// An unreachable retained boundary competes with a short legal completion
// through one transit-owned source. The old code floods 262144 gap states
// after its early free source probe fails, before trying that completion.
void gap_fallback_order_regression() {
  // Zero is the production-size blocked boundary; the other cases prove
  // that the shortcut cannot override free/shared entries or cut policy.
  for (int mode : {0, 1, 2, 3, 4}) {
    GapCostProfile profile;
    Fixture f(6, 102);
    auto first = f.edge({0, 1}, {1, 1}, 144, 200, 500, 0);
    first.owns_landing = true;
    Route r{f.pin({0, 1}, 144, true), first, gapCostHole()};
    f.terminal(r, {3, 3}, 700, 91);
    auto &route = f.install(841, std::move(r));
    f.tile({0, 1}).pin_state.leased_nodes.clearBit(144);
    if (mode == 0)
      gapCostFabric(f, 1);
    f.edge({1, 1}, {2, 3}, 500, 201, 600);
    f.edge({2, 3}, {3, 3}, 600, 222, 700);
    // This transit owns the sole physical SRC into the retained boundary,
    // but lands on a different DST: numeric jumps may have multiple landings.
    f.arc({2, 3, fpga::CB_NODE_SRC, 222}, {3, 3, fpga::CB_NODE_DST, 701});
    Route victim{f.pin({2, 5}, 3990, true), f.edge({2, 5}, {2, 3}, 3990, 3990, 800, 0)};
    auto v = f.edge({2, 3}, {3, 3}, 800, 223, 701);
    // Use the same physical SRC already declared with both landing bits.
    f.tile({2, 3}).cb_type->dst_src[800].jump.clearBit(223);
    f.tile({2, 3}).cb_type->dst_src[800].jump.setBit(222);
    v.jump = v.route_jump = 222;
    victim.push_back(v);
    victim.push_back(f.edge({3, 3}, {4, 3}, 701, 3990, 3990));
    f.terminal(victim, {4, 3}, 3990, 3990);
    if (mode == 2) {
      // A sibling owns the ingress SRC, so it cannot prove a foreign cut.
      // Preserve its exact incoming arc and independent terminal as well.
      victim.erase(victim.begin(), victim.begin() + 2);
      victim.insert(victim.begin(), f.edge({1, 1}, {2, 3}, 500, 202, 800));
      victim.insert(victim.begin(), first);
      victim.insert(victim.begin(), f.pin({0, 1}, 144, true));
      f.install(841, std::move(victim));
      f.arc({3, 3, fpga::CB_NODE_DST, 701}, {3, 3, fpga::CB_NODE_LOCAL, 91});
    } else {
      f.install(842, std::move(victim));
      f.tile({2, 5}).pin_state.leased_nodes.clearBit(3990);
    }
    if (mode == 1) {
      // One free physical predecessor forbids any shortcut/preemption even
      // though the other incoming SRC still has a valid transit victim.
      f.edge({2, 3}, {3, 3}, 600, 224, 700);
    }
    f.rebuild();
    f.device.rebuildIncomingDstMasks();
    const auto before = f.state();
    int commits = 0;
    auto preempt = [&](const std::vector<fpga::RouteCutNode> &cuts, bool commit,
                       std::vector<fpga::RouteCutNode> &released) {
      if (mode == 3)
        return false;
      if (cuts.size() != 1 || !same(cuts[0].tile, {2, 3}) || cuts[0].type != fpga::CB_NODE_SRC ||
          cuts[0].node != 222)
        return false;
      auto victims = fpga::findNetRouteCutVictims(cuts);
      for (const auto &owner : victims)
        require(owner.net == &f.signal(842).net, "completion selected a non-transit owner");
      released = fpga::previewNetRouteCuts(victims, cuts, true);
      if (!commit)
        return !released.empty();
      ++commits;
      return fpga::unrouteNetRoutesFromNodes(victims, cuts, true);
    };
    require(!fpga::repairNetRouteGaps(
                f.signal(841).net, 0, [] { return true; }, 262144, 200, {}, preempt) &&
                f.state() == before && !commits,
            "cancelled completion mutated routing");
    auto index = pnr::buildBackwardResolveIndex(f.device, {3, 3}, 102);
    const fpga::RepairIncomingSources incoming =
        [&](Coord at, fpga::CBNodeNameType kind, int node,
            const fpga::RepairIncomingSourceVisitor &visit) {
          pnr::visitBackwardPhysicalSources(index, at, kind, node, visit);
        };
    const fpga::RepairIncomingSources unknown = [](Coord, fpga::CBNodeNameType, int,
                                                   const fpga::RepairIncomingSourceVisitor &) {};
    // An absent/empty reverse query is not proof that every ingress is busy.
    const auto &lookup = mode == 4 ? unknown : incoming;
    require(!fpga::repairNetRouteGaps(
                f.signal(841).net, 0, [] { return true; }, 262144, 200, {}, preempt, lookup) &&
                f.state() == before && !commits,
            "cancelled reverse shortcut changed leases");
    fpga::resetRepairStatistics();
    const bool done =
        fpga::repairNetRouteGaps(f.signal(841).net, 0, {}, 262144, 200, {}, preempt, lookup);
    require(done == (mode != 3),
            "completion ignored the cut policy or missed the free/shared path");
    require(commits == int(mode == 0 || mode == 4) && fpga::isRouteComplete(route) == done,
            "completion made an unnecessary cut or failed to complete");
    // Check every numeric connection and continuity, not just a complete flag.
    if (done)
      check_complete_path(f, route, 3);
    if (!done)
      require(f.state() == before, "refused cut changed retained or victim routes");
    if (mode == 2)
      require(fpga::isRouteComplete(f.signal(841).sinks[1].wires.front()),
              "shared ingress repair damaged the sibling");
    f.state(); // Every remaining busy bit must still have its route owner.
    using C = fpga::RepairCounter;
    const auto stats = fpga::repairStatistics();
    std::cout << "gap fallback mode=" << mode << " complete=" << done << " cuts=" << commits
              << '\n';
    fpga::printRepairStatistics(std::cout, "gap fallback production scale");
    // Avoid the irrelevant flood, not the real source/ownership validation.
    if (mode == 0)
      require(stats[C::source_success] == 1 && stats[C::gap_expanded] == 0,
              "proven foreign ingress still flooded the gap before completion");
    if (mode == 1 || mode == 2 || mode == 4)
      require(stats[C::gap_expanded] > 0,
              "free/shared/unknown physical ingress was incorrectly pruned");
  }
}

// Native call 13574 spent 262144 gap expansions after its free probe failed,
// then its cached-source completion succeeded immediately. Unlike the earlier
// blocked-ingress fixture, a FREE physical ingress forbids that shortcut here:
// it is on a disconnected island, not reachable from the retained prefix.
// All names are generated by Fixture; every occupied bit has a real owner.
void gap_deferred_completion_regression(bool reproduce) {
  GapCostProfile profile;
  setenv("SCALEPNR_FREE_SEARCH_STATES", "65536", 1);
  Fixture f(6, 102);
  auto first = f.edge({0, 1}, {1, 1}, 144, 200, 500, 0);
  first.owns_landing = true;
  Route retained{f.pin({0, 1}, 144, true), first, gapCostHole()};
  f.terminal(retained, {3, 3}, 700, 91);
  auto &route = f.install(844, std::move(retained));
  f.tile({0, 1}).pin_state.leased_nodes.clearBit(144);
  // Preserve a large reachable component, with thirteen choices per arrival.
  // It cannot reach the retained destination without the transit cut below.
  gapCostFabric(f, 13);
  f.edge({1, 1}, {2, 3}, 500, 201, 600);
  f.edge({2, 3}, {3, 3}, 600, 222, 700);
  f.arc({2, 3, fpga::CB_NODE_SRC, 222}, {3, 3, fpga::CB_NODE_DST, 701});
  Route victim{f.pin({2, 5}, 3990, true),
               f.edge({2, 5}, {2, 3}, 3990, 3990, 800, 0)};
  // This is a transit arc, not the victim's takeoff or grounding connection.
  f.arc({2, 3, fpga::CB_NODE_DST, 800}, {2, 3, fpga::CB_NODE_SRC, 222});
  Wire transit;
  transit.from = {2, 3}; transit.to = {3, 3}; transit.pos = 1;
  transit.local = 800; transit.jump = transit.route_jump = 222; transit.dst = 701;
  transit.owns_dst = true;
  victim.push_back(transit);
  victim.push_back(f.edge({3, 3}, {4, 3}, 701, 3990, 3990));
  f.terminal(victim, {4, 3}, 3990, 3990);
  f.install(845, std::move(victim));
  f.tile({2, 5}).pin_state.leased_nodes.clearBit(3990);
  // A complete reverse index must report BOTH the occupied ingress and this
  // genuinely free one. No path from the requester reaches this island.
  f.edge({4, 4}, {3, 3}, 902, 225, 700);
  f.rebuild();
  f.device.rebuildIncomingDstMasks();
  auto index = pnr::buildBackwardResolveIndex(f.device, {3, 3}, 102);
  int free_ingress_queries = 0;
  const fpga::RepairIncomingSources incoming =
      [&](Coord at, fpga::CBNodeNameType kind, int node,
          const fpga::RepairIncomingSourceVisitor &visit) {
    pnr::visitBackwardPhysicalSources(index, at, kind, node,
        [&](Coord source, fpga::CBNodeNameType source_kind, int source_node) {
      if (same(at, {3, 3}) && kind == fpga::CB_NODE_DST && node == 700 &&
          same(source, {4, 4}) && source_kind == fpga::CB_NODE_SRC && source_node == 225) {
        require(!f.tile(source).cb.src.jump.testBit(source_node), "island ingress is not free");
        ++free_ingress_queries;
      }
      visit(source, source_kind, source_node);
    });
  };
  const auto before = f.state();
  using C = fpga::RepairCounter;
  int commits = 0;
  bool observed_failed_probe = false;
  auto preempt = [&](const std::vector<fpga::RouteCutNode> &cuts, bool commit,
                     std::vector<fpga::RouteCutNode> &released) {
    if (cuts.size() != 1 || !same(cuts[0].tile, {2, 3}) ||
        cuts[0].type != fpga::CB_NODE_SRC || cuts[0].node != 222)
      return false;
    if (!observed_failed_probe) {
      // Completed-call counters prove that FREE ONLY failed before completion
      // was allowed to preempt; speculative exploration must not alter leases.
      const auto &stats = fpga::repairStatistics();
      require(stats[C::source_calls] == 2 && stats[C::source_success] == 0 &&
                  stats[C::two_ended_calls] == 1 && stats[C::source_labels] >= 65536 &&
                  stats[C::source_labels] <= 65536 + 16,
              "fixture did not exhaust both bounded free proofs before transit completion");
      require(f.state() == before, "failed free/gap search changed route ownership");
      observed_failed_probe = true;
    }
    auto victims = fpga::findNetRouteCutVictims(cuts);
    require(victims.size() == 1 && victims.front().net == &f.signal(845).net,
            "completion did not select exactly the installed transit owner");
    released = fpga::previewNetRouteCuts(victims, cuts, true);
    if (!commit) return !released.empty();
    ++commits;
    return fpga::unrouteNetRoutesFromNodes(victims, cuts, true);
  };
  fpga::resetRepairStatistics();
  require(fpga::repairNetRouteGaps(f.signal(844).net, 0, {}, 262144, 200, {}, preempt, incoming),
          "completion fallback failed to repair the short route");
  // Require a complete numeric path, exactly one transit cut, and coherent
  // registrations/masks. Speed cannot come from silently dropping routing.
  require(free_ingress_queries > 0 && observed_failed_probe && commits == 1,
          "fixture skipped its free-ingress/probe/preemption prerequisites");
  check_complete_path(f, route, 3);
  f.state();
  const auto stats = fpga::repairStatistics();
  require(stats[C::source_calls] == 3 && stats[C::source_success] == 1 &&
              stats[C::two_ended_calls] == 1 &&
              stats[C::reused_frontiers] == 1 && stats[C::gap_success] == 1,
          "completion did not reuse the failed free frontier exactly once");
  fpga::printRepairStatistics(std::cout, "gap deferred completion with free island ingress");
  std::cout << "deferred completion: free_probe_labels=" << stats[C::source_labels]
            << " gap_expanded=" << stats[C::gap_expanded] << " transit_cuts=" << commits
            << " complete=1 reproduce=" << reproduce << '\n';
  if (reproduce) {
    // Characterization mode confirms the native bad ordering before any fix.
    require(stats[C::gap_expanded] == 262144 && stats[C::gap_edges] > 3000000,
            "did not reproduce the full native-sized gap flood before successful completion");
  } else {
    // The early free probe starts after 64 gap expansions. A successful
    // completion must not be postponed behind the remaining 262080 states.
    require(stats[C::gap_expanded] <= 64,
            "successful completion was deferred behind the full exact-gap flood");
  }
}

// The same neutral-name frontier fixture must retain its routing/lease
// assertions with counters both disabled and enabled. Reuse is not expansion.
void repair_statistics_regression() {
  struct ProfileEnvironment {
    bool present=std::getenv("SCALEPNR_PROFILE_REPAIR")!=nullptr;
    std::string value=present?std::getenv("SCALEPNR_PROFILE_REPAIR"):"";
    ~ProfileEnvironment() {
      if(present) setenv("SCALEPNR_PROFILE_REPAIR",value.c_str(),1);
      else unsetenv("SCALEPNR_PROFILE_REPAIR");
    }
  } environment;
  unsetenv("SCALEPNR_PROFILE_REPAIR");
  fpga::resetRepairStatistics();
  gap_source_frontier_reuse_regression();
  require(fpga::repairStatistics()[fpga::RepairCounter::gap_calls]==0,
      "disabled profiling recorded repair calls");
  setenv("SCALEPNR_PROFILE_REPAIR","1",1);
  gap_source_frontier_reuse_regression();
  using C=fpga::RepairCounter;
  const auto &stats=fpga::repairStatistics();
  // Four cancelled calls plus four real gap repairs; each real call probes
  // the source twice, reusing the first free frontier on the second call.
  require(stats[C::gap_calls]==8 && stats[C::cancelled]==4,
      "repair call/cancellation counts are not exact");
  require(stats[C::source_calls]==8 && stats[C::reused_frontiers]==4,
      "reused source frontiers were not counted exactly once");
  require(stats[C::gap_success]==2 && stats[C::source_success]==2,
      "repair success counters disagree with committed routes");
  // The gap and source traverse common nodes, but neither should expand its
  // own exhausted graph again. Duplicate rejected edges are a different count.
  require(stats[C::gap_expanded]==stats[C::gap_unique] &&
          stats[C::source_expanded]==stats[C::source_unique] &&
          stats[C::gap_reexpanded]==0 && stats[C::source_reexpanded]==0 &&
          stats[C::source_cross_call_repeat]==0,
      "frontier reuse was misreported as repeated expansion");
  require(stats[C::gap_source_overlap]>0 &&
          stats[C::gap_source_overlap]<=stats[C::gap_unique] &&
          stats[C::gap_source_overlap]<=stats[C::source_unique] &&
          stats[C::gap_parent_steps]>0 && stats[C::source_parent_steps]>0,
      "repair overlap or parent-chain work was not observed");
  std::ostringstream out;
  fpga::printRepairStatistics(out,"neutral regression");
  require(out.str().starts_with("REPAIR_TOTAL context=\"neutral regression\"") &&
          out.str().find("gap_calls=8")!=std::string::npos,
      "repair totals cannot be parsed from the log");
  std::cout << out.str();
  fpga::resetRepairStatistics();
  require(fpga::repairStatistics()[C::gap_calls]==0,"repair totals did not reset");
}

// Reproduce the 50K budget-retry restart without importing device names or
// topology: a shallow numeric tree has exactly 64,304 reachable states.
void source_budget_frontier_reuse_regression() {
  struct Environment {
    std::map<std::string,std::string> old;
    Environment() {
      for(const char *key:{"SCALEPNR_PROFILE_REPAIR","SCALEPNR_FREE_SEARCH_STATES"})
        if(const char *value=std::getenv(key)) old[key]=value;
      setenv("SCALEPNR_PROFILE_REPAIR","1",1);
      setenv("SCALEPNR_FREE_SEARCH_STATES","65536",1);
    }
    ~Environment() {
      for(const char *key:{"SCALEPNR_PROFILE_REPAIR","SCALEPNR_FREE_SEARCH_STATES"}) {
        auto it=old.find(key);
        if(it==old.end()) unsetenv(key); else setenv(key,it->second.c_str(),1);
      }
    }
  } environment;
  Fixture f(5,20);
  f.arc({0,0,fpga::CB_NODE_LOCAL,0},{0,0,fpga::CB_NODE_JOINT,0});
  f.arc({0,0,fpga::CB_NODE_JOINT,0},{0,0,fpga::CB_NODE_SRC,0});
  f.arc({0,0,fpga::CB_NODE_SRC,0},{1,0,fpga::CB_NODE_DST,0});
  for(int branch=0;branch<16;++branch) {
    f.edge({1,0},{branch+2,1},0,branch,0);
    for(int leaf=0;leaf<2008+(branch==0?6:0);++leaf) {
      f.edge({branch+2,1},{branch+2,2},0,leaf,leaf);
      f.arc({branch+2,2,fpga::CB_NODE_DST,leaf},{branch+2,2,fpga::CB_NODE_LOCAL,0});
    }
  }
  // Four root states, 32 branch states, 32,134 SRC/DST leaf pairs. Every
  // SRC has a real numeric landing leading to an unrelated terminal input;
  // terminal-input pruning excludes those LOCALs, not their incoming DSTs.
  constexpr size_t reachable=4+32+2*32134;
  static_assert(reachable==64304);
  auto &s=f.signal(730);
  const auto from=f.pin({0,0},0,true), to=f.pin({19,4},91,false);
  s.driver.tile.set(&f.device.tile_grid[0]);
  s.sinks.emplace_back(); s.sink_cells.emplace_back();
  auto &sink=s.sinks.back();
  s.sink_cells.back().name="receiver_730";
  s.sink_cells.back().type="logic_731";
  sink.cell_ref.set(&s.sink_cells.back()); sink.tile.set(&f.device.tile_grid[4*20+19]);
  rtl::NetRouteBinding binding;
  binding.from=&s.driver; binding.to=&sink;
  binding.from_port="out"; binding.to_port="in"; binding.route_name="branch_730";
  f.rebuild();
  const auto before=f.state();
  const auto revision=fpga::routeMutationRevision();
  fpga::UnboundRouteSearch search(s.net,binding);
  bool budget_limited=false;
  auto attempt=[&](bool reuse,const Wire &target,size_t budget,int hops=200,
                   const std::function<bool()> &cancel=std::function<bool()>{}) {
    RepairDecisionTrace trace;
    const bool done=reuse ? search.route(from,target,cancel,budget,hops,{}, {},&budget_limited)
        : fpga::routeUnboundNetFromSource(s.net,binding,from,target,cancel,budget,hops,{}, {},&budget_limited);
    std::istringstream records(trace.read());
    std::vector<size_t> expansions;
    for(std::string line;std::getline(records,line);) {
      if(!line.starts_with("REPAIR_CALL ")) continue;
      const auto at=line.find(" source_expanded=");
      expansions.push_back(at==std::string::npos ? 0 : std::stoull(line.substr(at+17)));
    }
    return std::pair{done,expansions};
  };
  for(bool reuse:{false,true}) {
    std::vector<size_t> capped_expansions;
    for(size_t budget:{65536,131072,524288}) {
      const auto [done,expansions]=attempt(reuse,to,budget);
      require(!done,"disconnected synthetic sink was reported routed");
      require(!budget_limited,"exhausted 64,304-state graph reported a cap rejection");
      require(expansions.size()>=2,"missing restricted/unrestricted search counters");
      capped_expansions.push_back(expansions[1]);
      std::cout << "budget-retry reuse=" << reuse << " budget=" << budget
          << " capped_expanded=" << expansions[1]
          << " revision=" << fpga::routeMutationRevision() << '\n';
      // The uncapped larger search is a separate opportunity, not a cached failure.
      if(budget>65536) require(expansions.back()==reachable,
          "frontier reuse incorrectly suppressed the uncapped larger search");
      // Failure must leave every allocation and route registration unchanged.
      require(f.state()==before && s.net.routes.empty() && sink.wires.empty() &&
          fpga::routeMutationRevision()==revision,"failed budget retry changed ownership");
    }
    require(capped_expansions.front()==reachable,"fixture did not expand exactly 64,304 states");
    // Keep the old API as a positive reproduction control, then verify the fix.
    require(capped_expansions[1]==(reuse?0:reachable) &&
        capped_expansions[2]==(reuse?0:reachable),
        "BUG: unchanged 64,304-state free frontier was expanded three times");
  }
  // Another numeric endpoint must not inherit the previous pin's failed graph.
  const auto alternative=f.pin({19,4},92,false);
  require(attempt(true,alternative,65536).second.at(1)==reachable,
      "different sink pin reused the old frontier");
  // Hop and free-label bounds change the graph; the unchanged request remains reusable.
  require(attempt(true,to,65536,1).second.at(1)==20,"hop bound reused a longer frontier");
  require(attempt(true,to,65536).second.at(1)==0,"different pin/hop probe discarded reusable frontier");
  setenv("SCALEPNR_FREE_SEARCH_STATES","32768",1);
  require(attempt(true,to,32768).second.at(1)==32768,"changed cap reused a larger frontier");
  require(budget_limited,"truncated free frontier reported exhaustion");
  require(attempt(true,to,32768).second.at(1)==0 && budget_limited,
      "cached capped frontier lost its limit flag and would suppress later budgets");
  setenv("SCALEPNR_FREE_SEARCH_STATES","65536",1);
  // A cancelled partial search is not an exhausted reusable frontier.
  search.clear(); int polls=0;
  require(!attempt(true,to,65536,200,[&]{return ++polls>100;}).first,
      "cancelled source request succeeded");
  require(attempt(true,to,65536).second.at(1)==reachable,"cancelled frontier was reused");
  require(f.state()==before,"cancelled source request leaked ownership");
  // Five large pin-specific frontiers exceed the label bound: retain the
  // recent request, but rebuild an evicted one rather than growing indefinitely.
  search.clear();
  for(int pin=100;pin<105;++pin)
    require(attempt(true,f.pin({19,4},pin,false),65536).second.at(1)==reachable,
        "different input unexpectedly reused a frontier");
  require(attempt(true,f.pin({19,4},104,false),65536).second.at(1)==0,
      "recent frontier was needlessly evicted");
  require(attempt(true,f.pin({19,4},100,false),65536).second.at(1)==reachable,
      "frontier cache exceeded its label bound");
  require(attempt(true,to,65536).second.at(1)==reachable,"failed to warm occupancy-control frontier");
  // Install a real foreign owner, rather than directly setting a busy bit.
  Route victim{f.pin({18,0},90,true),f.edge({18,0},{2,2},90,90,0,0)};
  f.terminal(victim,{2,2},0,0); f.install(731,std::move(victim)); f.rebuild();
  // This fixture has no placement-owned source reservation; cb.local retains
  // the route's actual LOCAL lease, as in the other synthetic transit fixtures.
  f.tile({18,0}).pin_state.leased_nodes.clearBit(90);
  const auto busy=f.state();
  require(attempt(true,to,65536).second.at(1)==reachable-1 && f.state()==busy,
      "route mutation reused the stale free landing");
  require(fpga::unrouteNet(f.signal(731).net),"cannot release fixture owner");
  require(attempt(true,to,65536).second.at(1)==reachable,
      "released landing remained blacklisted by the request cache");
  f.state(); // Every remaining busy bit must still have a real route owner.
  // Placement changes explicitly start a fresh snapshot. A capped failure
  // must not suppress the successful larger search to this reachable input.
  search.clear(); sink.tile.set(&f.device.tile_grid[2*20+2]);
  const auto reachable_pin=f.pin({2,2},0,false);
  setenv("SCALEPNR_FREE_SEARCH_STATES","4",1);
  require(!attempt(true,reachable_pin,4).first,"small cap unexpectedly reached the sink");
  require(attempt(true,reachable_pin,65536).first,"cached cap prevented larger successful search");
  require(s.net.routes.size()==1 && sink.wires.size()==1 &&
      fpga::isRouteComplete(sink.wires.front()),"successful retry left a partial route");
  check_complete_path(f,sink.wires.front(),3);
  f.state();
  require(fpga::unrouteNet(s.net),"cannot release successful retry");
  f.state(); // Teardown must free the requester without leaving orphan leases.
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

// A retained hole must honor the same explicit recovery budget as an unbound
// connection. The LOCAL relay prevents exact-gap DST traversal from masking
// the capped source-completion defect seen during the fixed-endpoint repair.
void retained_gap_budget_escalation_regression() {
  const char *key="SCALEPNR_FREE_SEARCH_STATES";
  const bool present=std::getenv(key)!=nullptr;
  const std::string original=present?std::getenv(key):"";
  struct Restore {
    const char *key; bool present; std::string value;
    ~Restore() { if(present) setenv(key,value.c_str(),1); else unsetenv(key); }
  } restore{key,present,original};
  for (bool exact_gap : {false,true}) for (int length : {12,24,48})
      for (int cap : {4,16}) for (bool offer_preemption : {false,true}) {
    setenv(key,std::to_string(cap).c_str(),1);
    Fixture f(6,length+2);
    auto first=f.edge({1,2},{2,2},144,200,500,0);
    first.owns_landing=true;
    Wire gap;
    gap.type=Wire::WIRE_GAP;
    gap.from={2,2}; gap.from_node_type=fpga::CB_NODE_DST; gap.from_node=500;
    gap.to={length,2}; gap.to_node_type=fpga::CB_NODE_LOCAL; gap.to_node=91;
    auto &route=f.install(917,{f.pin({1,2},144,true),first,gap,f.pin({length,2},91,false)});
    f.tile({1,2}).pin_state.leased_nodes.clearBit(144);
    auto &signal=f.signal(917);
    signal.driver.outline.fixed=signal.sinks.front().outline.fixed=true;
    for(int x=2;x<length;++x) f.edge({x,2},{x+1,2},500,200,500);
    const int middle=length/2;
    f.tile({middle,2}).cb_type->dst_src[500].jump.clearBit(200);
    f.arc({middle,2,fpga::CB_NODE_DST,500},{middle,2,fpga::CB_NODE_LOCAL,180});
    f.arc({middle,2,fpga::CB_NODE_LOCAL,180},{middle,2,fpga::CB_NODE_SRC,200});
    f.arc({length,2,fpga::CB_NODE_DST,500},{length,2,fpga::CB_NODE_JOINT,54});
    f.arc({length,2,fpga::CB_NODE_JOINT,54},{length,2,fpga::CB_NODE_JOINT,39});
    f.arc({length,2,fpga::CB_NODE_JOINT,39},{length,2,fpga::CB_NODE_LOCAL,91});
    // Every occupied alternative belongs to a real independently routed signal.
    Route other{f.pin({length,4},3990,true),
        f.edge({length,4},{length,2},3990,3990,501,0),
        f.edge({length,2},{length,1},501,3990,3990)};
    f.terminal(other,{length,1},3990,3990);
    f.install(918,std::move(other));
    f.tile({length,4}).pin_state.leased_nodes.clearBit(3990);
    f.arc({length,2,fpga::CB_NODE_DST,501},{length,2,fpga::CB_NODE_JOINT,39});
    f.rebuild(); f.device.rebuildIncomingDstMasks();
    const auto before=f.state();
    fpga::RepairGroundingPreemption preempt;
    if(offer_preemption) preempt=[](const auto &,bool commit,auto &) {
      require(!commit,"free retained repair tried to preempt a transit"); return false;
    };
    auto repair=[&](size_t budget,const std::function<bool()> &cancel={}) {
      return exact_gap ? fpga::repairNetRouteGaps(signal.net,0,cancel,budget,100,{},preempt)
          : fpga::completeNetRouteFromSource(signal.net,0,cancel,budget,100,preempt);
    };
    // Respect the caller's small bound and preserve every retained lease on failure.
    require(!repair(cap) && f.state()==before,"small retained budget changed ownership");
    // A cancelled recovery must not use the later larger-budget attempt.
    require(!repair(4096,[]{return true;}) && f.state()==before,"cancelled retained repair mutated state");
    auto &terminal=f.tile({length,2}).cb_type->joint_local[39].local;
    terminal.clearBit(91); f.rebuild();
    // Increasing the bound must not invent a disconnected endpoint or borrow a transit.
    require(!repair(4096) && f.state()==before,"disconnected retained repair mutated state");
    terminal.setBit(91); f.rebuild();
    std::ostringstream witness;
    // Independently establish that the same retained route and leases admit a free completion.
    require(fpga::auditRouteCompletion(signal.net,signal.net.routes[0],route.front(),route.back(),
        witness,{},4096,4096,100) && f.state()==before,"read-only witness failed or mutated leases");
    std::ostringstream owner_before; fpga::dumpNetRouteHistory(f.signal(918).net,owner_before);
    RepairDecisionTrace trace;
    const bool done=repair(4096);
    std::printf("retained-budget exact=%d length=%d cap=%d preemption=%d complete=%d\n",
        exact_gap,length,cap,offer_preemption,done);
    // A retained binding must not silently repeat the fast cap at the larger caller budget.
    require(done,"BUG: retained-gap completion never uses the caller's full recovery budget");
    const auto decisions=trace.read();
    // Verify that the initial fast miss was followed by an explicit-budget success.
    require(decisions.find("limit="+std::to_string(cap)+" reached=0")!=std::string::npos,
        "retained regression did not exercise the fast cap");
    if(!offer_preemption)
      require(decisions.find("limit=4096 reached=1")!=std::string::npos,
          "retained completion did not honor the explicit caller budget");
    check_complete_path(f,route,length-1);
    require(signal.driver.outline.fixed && signal.sinks.front().outline.fixed,
        "wire-only completion changed fixed placements");
    std::ostringstream owner_after; fpga::dumpNetRouteHistory(f.signal(918).net,owner_after);
    require(owner_before.str()==owner_after.str(),"retained repair damaged foreign ownership");
    f.state(); // Audit every committed node against its real owner.
    require(fpga::unrouteNet(signal.net),"retained repair could not be unrouted");
    f.state(); // Teardown must preserve foreign leases and clear private leases.
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
  for(int cap:{4,8}) for(size_t budget:{64,128}) for(bool allow_cut:{false,true})
      for(bool reuse:{false,true}) {
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
    fpga::UnboundRouteSearch search(s.net,binding);
    auto run=[&](const std::function<bool()> &cancel,size_t limit) {
      return reuse ? search.route(from,to,cancel,limit,100,preempt)
          : fpga::routeUnboundNetFromSource(s.net,binding,from,to,cancel,limit,100,preempt);
    };
    // Reused free frontiers must retain the same real victim and commit policy.
    // Cancellation and the small explicit bound must leave every owner intact.
    require(!run([]{return true;},budget)
        && f.state()==before && !commits,"cancelled grounding escalation mutated routing");
    require(!run({},cap)
        && f.state()==before && !commits,"small-budget grounding escalation mutated routing");
    RepairDecisionTrace trace;
    const bool done=run({},budget);
    std::cout << "grounding-budget escalation cap=" << cap << " budget=" << budget
        << " allow_cut=" << allow_cut << " reuse=" << reuse << " complete=" << done << '\n' << trace.read();
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

// Fanouts' expensive tail repeatedly filled the exact-gap state table, then
// scanned every remaining queued state's outgoing wires/owners even though
// no new label could fit. The captured stage made 19 million such capacity
// rejections. This abstract fan-in fills 64 labels at the first expansion;
// its 63 queued arrivals each expose 128 more exits. Keep those real free
// arcs: failure must be read-only, and a goal already queued at saturation
// must still succeed, including a two-JOINT terminal Grounding.
void gap_saturated_frontier_regression() {
  struct ProfileScope {
    bool had=std::getenv("SCALEPNR_PROFILE_REPAIR")!=nullptr;
    std::string old=had?std::getenv("SCALEPNR_PROFILE_REPAIR"):"";
    ProfileScope() { setenv("SCALEPNR_PROFILE_REPAIR","1",1); }
    ~ProfileScope() { if(had)setenv("SCALEPNR_PROFILE_REPAIR",old.c_str(),1);
                     else unsetenv("SCALEPNR_PROFILE_REPAIR"); }
  } profile;
  for(int mode=0;mode<4;++mode) {
    Fixture f(8,8);
    const Coord target=mode?Coord{2,0}:Coord{7,7};
    Route r{f.pin({0,0},100,true),f.edge({0,0},{1,0},100,101,102,0)};
    r.back().owns_landing=true;
    Wire gap;gap.type=Wire::WIRE_GAP;gap.from={1,0};gap.from_node_type=fpga::CB_NODE_DST;
    gap.from_node=102;gap.to=target;
    gap.to_node_type=mode>=2?fpga::CB_NODE_LOCAL:fpga::CB_NODE_DST;
    gap.to_node=mode>=2?90:1003;r.push_back(gap);
    if(mode>=2) r.push_back(f.pin(target,90,false));
    else f.terminal(r,target,1003,90);
    auto &route=f.install(9701,std::move(r));
    for(int n=0;n<256;++n) f.edge({1,0},{2,0},102,1000+n,1000+n);
    for(int n=0;n<63;++n) for(int j=0;j<128;++j)
      f.edge({2,0},{3,0},1000+n,2000+j,2000+j);
    if(mode>=2) {
      f.arc({2,0,fpga::CB_NODE_DST,1003},{2,0,fpga::CB_NODE_JOINT,31});
      f.arc({2,0,fpga::CB_NODE_JOINT,31},{2,0,fpga::CB_NODE_JOINT,32});
      if(mode==2) f.arc({2,0,fpga::CB_NODE_JOINT,32},{2,0,fpga::CB_NODE_LOCAL,90});
    }
    f.rebuild();const auto before=f.state();fpga::resetRepairStatistics();
    const bool done=fpga::repairNetRouteGaps(f.signal(9701).net,0,{},64,200);
    const auto stats=fpga::repairStatistics();
    const bool expected=mode==1 || mode==2;
    require(done==expected,"state saturation skipped a queued goal or invented a Grounding");
    if(expected) { require(fpga::isRouteComplete(route),"queued goal left a partial route");
                   check_complete_path(f,route,2); }
    else require(f.state()==before,"failed saturated gap search changed leases or registrations");
    std::printf("Saturated gap: mode=%d complete=%d labels=%llu edges=%llu parent_steps=%llu\n",
        mode,done,(unsigned long long)stats[fpga::RepairCounter::gap_labels],
        (unsigned long long)stats[fpga::RepairCounter::gap_edges],
        (unsigned long long)stats[fpga::RepairCounter::gap_parent_steps]);
    require(stats[fpga::RepairCounter::gap_labels]<=64,"gap search exceeded its label bound");
    require(stats[fpga::RepairCounter::gap_edges]<=256,
        "full gap state table still scans queued outgoing edges that cannot be admitted");
    f.state();
  }
}

// The long failed Fanouts probes fill their source-label table, then keep
// scanning outgoing arcs from queued states that cannot add another label.
// Preserve those queued states: a sink or an intact retained tail may already
// be in the queue. All names below describe abstract nodes, not device names.
void source_saturated_frontier_work_regression(bool reproduce) {
  using C = fpga::RepairCounter;
  for (int budget : {64, 128, 256}) for (int mode = 0; mode < 4; ++mode) {
    GapCostProfile profile;
    setenv("SCALEPNR_FREE_SEARCH_STATES", std::to_string(budget).c_str(), 1);
    Fixture f(8, 8);
    f.node_pattern = std::format("source_cap{}_m{}", budget, mode);
    const Coord source{0,0}, target = mode ? source : Coord{7,7};
    const int branches = mode == 0 ? 4 * budget : budget - (mode == 2 ? 1 : 2);
    for (int n = 0; n < branches; ++n) {
      f.arc({0,0,fpga::CB_NODE_LOCAL,50},{0,0,fpga::CB_NODE_JOINT,1000+n});
      for (int j = 0; j < 128; ++j)
        f.arc({0,0,fpga::CB_NODE_JOINT,1000+n},{0,0,fpga::CB_NODE_SRC,3000+j});
    }
    for (int j = 0; j < 128; ++j)
      f.arc({0,0,fpga::CB_NODE_SRC,3000+j},{1,0,fpga::CB_NODE_DST,3000+j});
    if (mode == 1)
      f.arc({0,0,fpga::CB_NODE_LOCAL,50},{0,0,fpga::CB_NODE_LOCAL,90});
    if (mode == 3) {
      f.arc({0,0,fpga::CB_NODE_LOCAL,50},{0,0,fpga::CB_NODE_JOINT,3990});
      f.arc({0,0,fpga::CB_NODE_JOINT,3990},{0,0,fpga::CB_NODE_LOCAL,90});
    }
    const int join = 1000 + branches - 1;
    if (mode == 2)
      f.arc({0,0,fpga::CB_NODE_JOINT,join},{0,0,fpga::CB_NODE_LOCAL,90});
    Wire hole;
    hole.type = Wire::WIRE_GAP;
    hole.from = source; hole.from_node_type = fpga::CB_NODE_LOCAL; hole.from_node = 50;
    hole.to = target; hole.to_node_type = mode == 2 ? fpga::CB_NODE_JOINT : fpga::CB_NODE_LOCAL;
    hole.to_node = mode == 2 ? join : 90;
    Route request{f.pin(source,50,true), hole};
    if (mode == 2) {
      Wire tail;
      tail.type = Wire::WIRE_ROUTE_EDGE;
      tail.from = tail.to = source;
      tail.from_node_type = fpga::CB_NODE_JOINT; tail.from_node = join;
      tail.to_node_type = fpga::CB_NODE_LOCAL; tail.to_node = 90;
      request.push_back(tail);
    }
    request.push_back(f.pin(target,90,false));
    auto &route = f.install(9705,std::move(request));
    if (mode == 3) {
      Route owner{f.pin({0,2},70,true), f.edge({0,2},source,70,70,70,0)};
      f.terminal(owner,source,70,91,3990);
      f.install(9706,std::move(owner));
    }
    f.rebuild();
    const auto before = f.state();
    const auto revision = fpga::routeMutationRevision();
    fpga::resetRepairStatistics();
    require(!fpga::completeNetRouteFromSource(f.signal(9705).net,0,[]{return true;},budget,200,{}),
            "cancelled capped source search succeeded");
    require(fpga::repairStatistics()[C::source_expanded] == 0 && f.state() == before,
            "cancelled capped source search expanded states or changed owners");
    fpga::resetRepairStatistics();
    const bool done = fpga::completeNetRouteFromSource(f.signal(9705).net,0,{},budget,200,{});
    const auto stats = fpga::repairStatistics();
    require(done == (mode == 1 || mode == 2),
            "source saturation skipped a queued sink/tail or crossed a foreign Grounding");
    if (done) {
      require(fpga::isRouteComplete(route),"saturated source repair left a GAP");
      check_complete_path(f,route,0);
    } else require(f.state() == before && fpga::routeMutationRevision() == revision,
                   "failed capped source probe changed ownership");
    require(stats[C::source_labels] <= uint64_t(budget) &&
            stats[C::source_expanded] >= uint64_t(budget - 1),
            "work improvement changed the label budget or skipped queued states");
    std::printf("SOURCE_SATURATED budget=%d mode=%d complete=%d labels=%llu "
                "expanded=%llu edges=%llu cap_rejects=%llu\n",budget,mode,done,
        (unsigned long long)stats[C::source_labels],
        (unsigned long long)stats[C::source_expanded],
        (unsigned long long)stats[C::source_edges],
        (unsigned long long)stats[C::source_limit]);
    if (reproduce) {
      if (mode == 0) require(stats[C::source_edges] >= uint64_t((budget - 1) * 128),
                            "fixture did not reproduce post-saturation edge scans");
    } else require(stats[C::source_edges] <= uint64_t(4 * budget + 256),
                   "full source label table still scans unrecordable continuations");
    f.state();
  }
}

// Minimize the profiled Fanouts cost: many free arrivals in one Tile can each
// see many busy SRCs, while that Tile also registers unrelated transit nets.
// Every busy bit below comes from an installed complete route. The old
// sharedRepairEdge walks every registered binding separately for every SRC.
// Vary both dimensions so a smaller search budget cannot hide that product.
void busy_src_eligibility_work_regression(bool reproduce) {
  using C = fpga::RepairCounter;
  struct Work {
    int arrivals, exits, mode;
    uint64_t checks, bindings, sources, fragments;
  };
  std::vector<Work> measured;
  for (int scale = 0; scale < 3; ++scale) for (int mode = 0; mode < 4; ++mode) {
    GapCostProfile profile;
    const int arrivals = 8 << scale, exits = 32 << scale;
    const bool shared = mode >= 2, alias = mode == 3;
    const bool expected_complete = mode != 1;
    Fixture f(4, 6);
    f.node_pattern = std::format("busy_s{}_m{}", scale, mode);
    const Coord source{0,1}, start{1,1}, hub{2,1}, out{3,1}, finish{4,1};
    constexpr int requester = 9703;
    auto prefix = f.edge(source, start, 50, 50, 500, 0);
    prefix.owns_landing = true;
    Wire hole;
    hole.type = Wire::WIRE_GAP; hole.from = start; hole.to = finish;
    hole.from_node_type = hole.to_node_type = fpga::CB_NODE_DST;
    hole.from_node = 500; hole.to_node = 700;
    Route request{f.pin(source, 50, true), prefix, hole};
    f.terminal(request, finish, 700, 80);
    f.name(finish, fpga::CB_NODE_DST, 700);
    auto &route = f.install(requester, std::move(request));
    // Put the shared branch's actual incoming DST last. Reusing its SRC from
    // the first, different arrival must never pass the functional controls.
    for (int n = 0; n < arrivals; ++n) {
      const int dst = 600 + ((n + 1) % arrivals);
      f.edge(start, hub, 500, 200 + n, dst);
      for (int lane = 0; lane < exits; ++lane)
        f.arc({hub.x,hub.y,fpga::CB_NODE_DST,dst},
              {hub.x,hub.y,fpga::CB_NODE_SRC,1000 + lane});
      if (mode == 0) f.edge(hub, finish, dst, 3000, 700);
    }
    auto install_foreign = [&](int id, int node, Coord origin) {
      Route owner{f.pin(origin, node, true), f.edge(origin, hub, node, node, node, 0),
                  f.edge(hub, out, node, node, node)};
      f.terminal(owner, out, node, node);
      f.install(id, std::move(owner));
    };
    const int shared_src = 1000 + exits - 1;
    for (int lane = 0; lane < exits; ++lane) {
      if (!shared || lane != exits - 1)
        install_foreign(10000 + lane, 1000 + lane, {2,0});
      // These real owner bindings do not own any SRC tested by the requester.
      install_foreign(20000 + lane, 2000 + lane, {2,2});
    }
    Route *sibling = nullptr;
    if (shared) {
      Route branch{f.pin(source, 50, true), prefix,
                   f.edge(start, hub, 500, 200 + arrivals - 1, 600),
                   f.edge(hub, out, 600, shared_src, shared_src)};
      branch[0].shared = branch[1].shared = true;
      f.terminal(branch, out, shared_src, 3500);
      sibling = &f.install(alias ? 9704 : requester, std::move(branch));
      if (alias) {
        f.signal(9704).net.routes.front().from = &f.signal(requester).driver;
        f.signal(9704).net.invalidateRouteLookup();
      }
      f.edge(out, finish, shared_src, 3001, 700);
    }
    f.rebuild(); f.device.rebuildIncomingDstMasks();
    require(f.tile(hub).routed_bindings_authoritative &&
            f.tile(hub).routed_bindings.size() == size_t(2 * exits),
            "busy-SRC fixture lost its authoritative real-owner population");
    for (int lane = 0; lane < exits; ++lane) {
      const auto *name = f.tile(hub).cb_type->nodeName(fpga::CB_NODE_SRC, 1000 + lane);
      require(name && name->starts_with(f.node_pattern), "busy SRC lacks its patterned node name");
    }
    auto route_signature = [](const Route &wires) {
      std::ostringstream out;
      for (const auto &w : wires)
        out << w.type << ':' << w.from.x << ',' << w.from.y << ':'
            << w.to.x << ',' << w.to.y << ':' << w.local << ':' << w.jump
            << ':' << w.dst << ':' << w.joint << ':' << w.joint2 << ':'
            << w.pos << ':' << w.shared << ':' << w.owns_dst << ':' << w.owns_landing << '\n';
      return out.str();
    };
    auto other_history = [&] {
      std::ostringstream history;
      for (const auto &[id, signal] : f.signals)
        if (id != requester) fpga::dumpNetRouteHistory(signal->net, history);
      return history.str();
    };
    const auto before = f.state(), owners_before = other_history();
    const auto sibling_before = sibling ? route_signature(*sibling) : std::string{};
    fpga::resetRepairStatistics();
    require(!fpga::repairNetRouteGaps(f.signal(requester).net, 0, [] { return true; }, 4096, 100) &&
            fpga::repairStatistics()[C::shared_binding_visits] == 0 && f.state() == before,
            "cancelled busy-SRC search scanned bindings or modified real owners");
    fpga::resetRepairStatistics();
    const bool complete = fpga::repairNetRouteGaps(f.signal(requester).net, 0, {}, 4096, 100);
    const auto stats = fpga::repairStatistics();
    require(complete == expected_complete, "busy-SRC search changed routing feasibility");
    const uint64_t expected_checks = uint64_t(arrivals) * exits + unsigned(shared);
    require(stats[C::shared_edge_checks] == expected_checks &&
            stats[C::gap_expanded] == uint64_t(arrivals + (shared ? 3 : mode == 0 ? 2 : 1)) &&
            stats[C::gap_limit] == 0 && stats[C::gap_busy_src] >= uint64_t(arrivals) * (exits - 1),
            "busy-SRC work was hidden by pruning arrivals, exits, or the state budget");
    require(other_history() == owners_before &&
            (!sibling || route_signature(*sibling) == sibling_before),
            "busy-SRC search damaged a foreign transit or retained shared branch");
    if (complete) {
      check_complete_path(f, route, shared ? 4 : 3);
      if (shared) require(std::any_of(route.begin(), route.end(), [&](const Wire &w) {
        return w.type == Wire::WIRE_CROSSBAR && same(w.from, hub) &&
               w.jump == shared_src && w.local == 600 && w.shared;
      }), "busy-SRC search reused the shared exit from the wrong incoming DST");
    } else require(f.state() == before, "failed busy-SRC search changed leases or registrations");
    f.state();
    require(stats[C::shared_binding_visits] > 0 &&
            stats[C::shared_source_checks] <= stats[C::shared_binding_visits],
            "busy-SRC work counters did not observe real binding inspections");
    measured.push_back({arrivals, exits, mode, expected_checks, stats[C::shared_binding_visits],
                        stats[C::shared_source_checks], stats[C::shared_fragment_checks]});
    std::printf("BUSY_SRC_WORK arrivals=%d exits=%d owners=%d mode=%d complete=%d "
                "checks=%llu binding_visits=%llu source_checks=%llu fragment_checks=%llu limit=%llu\n",
        arrivals, exits, 2 * exits, mode, complete,
        (unsigned long long)expected_checks, (unsigned long long)stats[C::shared_binding_visits],
        (unsigned long long)stats[C::shared_source_checks],
        (unsigned long long)stats[C::shared_fragment_checks], (unsigned long long)(4 * expected_checks));
  }
  // Characterization must demonstrate the original owners-per-SRC product.
  // The ordinary regression demands roughly 16/32/64 times fewer binding
  // visits at the three scales, without changing any coverage/ownership checks.
  for (const auto &w : measured) {
    if (reproduce) require(w.bindings >= uint64_t(w.arrivals) * w.exits * (2 * w.exits),
                          "busy-SRC fixture did not reproduce repeated full Tile-binding scans");
    else require(w.bindings <= 4 * w.checks && w.sources <= 4 * w.checks &&
                     w.fragments <= uint64_t(16 * (w.arrivals + 3)),
                 "busy-SRC eligibility exceeds its binding/source/fragment work bounds");
  }
}

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
