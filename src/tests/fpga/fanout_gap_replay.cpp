#include "RegBunch.h"
#include "TimingPath.h"
#include "Device.h"
#include "Wire.h"
#include "route/RouteDesign.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <deque>
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

  explicit Fixture(int height = 40) {
    device.tile_grid.clear();
    device.cb_types.clear();
    device.cb_types.reserve(256);
    device.grid_spec.size = {40, height};
    device.size_width = 40; device.size_height = height;
    device.tile_grid.resize(40 * height);
    for (int y = 0; y < height; ++y) for (int x = 0; x < 40; ++x) {
      auto &t = device.tile_grid[y * 40 + x];
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
      t->full_name = "tile_" + std::to_string(c.y * 40 + c.x);
      tiles.push_back(t);
    }
    return *t;
  }

  void name(Coord c, fpga::CBNodeNameType kind, int id) {
    tile(c).cb_type->rememberNodeName(kind, id,
        "node_" + std::to_string((static_cast<unsigned>(id) * 2654435761U) ^
                                  (static_cast<unsigned>(kind) * 101U)));
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
    if (s.sinks.empty()) s.driver.tile.set(&device.tile_grid[r.front().from.y*40+r.front().from.x]);
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
    sink.tile.set(&device.tile_grid[r.back().from.y*40+r.back().from.x]);
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

  std::string state() {
    std::ostringstream out;
    std::vector<rtl::Net *> nets;
    for (const auto &[id,s] : signals) {
      nets.push_back(&s->net);
      fpga::dumpNetRouteHistory(s->net,out);
    }
    for (auto *t : tiles) {
      const auto a=fpga::auditTileCongestion(*t,out,nets);
      if (a.orphan_leases || a.missing_leases || a.missing_registrations || a.stale_registrations)
        throw std::runtime_error("fixture contains orphan leases or broken route registrations\n"+out.str());
    }
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
enum class FinalJumpGuard { None, Takeoff, Grounding, Joint, Intermediate };
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
  const Coord landing=guard==FinalJumpGuard::Intermediate?Coord{15,28}:Coord{15,29};
  f.edge({15,30},landing,193,247,241);
  if (guard==FinalJumpGuard::Intermediate)
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
  require(!fpga::repairNetRouteGaps(s.net,0,{},32768,7,{},reject_preview) &&
          previews==0 && f.state()==before,"final jump bypassed the total hop bound");
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
  run_final_jump_replay(false,FinalJumpGuard::Intermediate);
  run_final_jump_replay();
  run_final_jump_replay(true);
  run_final_jump_replay(false,FinalJumpGuard::None,true);
}

// Reproduce the next recorded failure: a reached, free DST has a direct arc
// to this binding's own sink, but the old two-JOINT input remains protected.
// Numeric coordinates/arcs are captured; all textual names are synthetic.
enum class SinkGuard { None, Shared, Alias, Foreign };
static bool run_private_grounding_replay(bool retain_grounding, SinkGuard guard=SinkGuard::None) {
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
    // Isolate foreign ownership at the sink, not at every source-prefix node.
    if (guard==SinkGuard::Foreign) preserved.back().shared=false;
    const int id=guard==SinkGuard::Shared?162316:162317;
    f.install(id,std::move(preserved));
    if (guard==SinkGuard::Alias) f.signal(id).net.routes.front().from=&s.driver;
  }

  // Last two hops and intermediate LOCALs of the actual six-hop alternative.
  f.name({5,37},fpga::CB_NODE_LOCAL,128);
  f.name({5,37},fpga::CB_NODE_LOCAL,184);
  f.tile({5,37}).cb_type->dst_local[3379].local.setBit(128);
  f.tile({5,37}).cb_type->local_local[128].local.setBit(184);
  f.edge({5,37},{11,37},184,3078,1030,0);
  f.edge({11,37},{12,37},1030,260,263);
  auto &sink=f.tile({12,37});
  sink.cb_type->dst_local[263].local.setBit(111);
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
  const auto before=f.state();
  std::ostringstream strict,relaxed;
  const bool strict_reachable=fpga::auditNetRouteReachability(s.net,0,strict);
  require(strict_reachable==(guard==SinkGuard::None),
          "strict reachability disagrees with private/shared terminal policy");
  require(fpga::auditNetRouteReachability(s.net,0,relaxed,false)==(guard!=SinkGuard::Foreign) &&
          relaxed.str().find("limited=1")==std::string::npos,
          "independent audit missed the captured direct completion");
  // Without the direct final arc even relaxed input rules cannot reach it.
  sink.cb_type->dst_local[263].local.clearBit(111);
  std::ostringstream broken;
  require(!fpga::auditNetRouteReachability(s.net,0,broken,false),
          "fixture contains an accidental alternate sink path");
  require(!fpga::repairNetRouteGaps(s.net,0) && f.state()==before,
          "repair crossed a missing final arc or changed failed routing");
  sink.cb_type->dst_local[263].local.setBit(111);
  require(f.state()==before,"read-only topology probes changed leases or routes");
  require(!fpga::repairNetRouteGaps(s.net,0,[]{return true;}) && f.state()==before,
          "cancelled private sink repair changed routing");
  require(!fpga::repairNetRouteGaps(s.net,0,{},32768,5) && f.state()==before,
          "private sink repair bypassed the total hop bound");
  auto &bridge=f.tile({5,37});
  bridge.input_joint_reservations_initialized=true;
  bridge.input_local_reservations.emplace(128,nullptr);
  const auto reserved=f.state();
  require(!fpga::repairNetRouteGaps(s.net,0) && f.state()==reserved,
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
    require(guard!=SinkGuard::None && !strict_reachable && f.state()==before,
            "private sink still rejected, or protected sink rejection changed routing");
    std::ifstream in(trace.path); std::string line; bool reached=false, limited=false;
    while(std::getline(in,line)) {
      if(line.starts_with("SOURCE_SEARCH_LIMIT ")) limited=true;
      if(line.starts_with("SOURCE_SEARCH_NODE ") &&
         line.find("tile=12,37 kind="+std::to_string(fpga::CB_NODE_DST)+" node=263 ")!=std::string::npos &&
         line.find("hops=6 expanded=1")!=std::string::npos) reached=true;
    }
    require(reached && !limited,"production did not reach and expand the recorded free six-hop arrival");
    require(!fpga::repairNetRouteGaps(s.net,0,{},65536,200) && f.state()==before,
            "larger budget changed the private-terminal failure");
    std::printf("private Grounding: preserved guard=%d rejected unchanged; reached six-hop DST263; ownership clean\n",int(guard));
  } else {
    require(guard==SinkGuard::None,"repair rewrote a preserved terminal input");
    check_complete_path(f,route,6);
    require(route[route.size()-2].local==263 && route[route.size()-2].joint<0 &&
            route[route.size()-2].joint2<0 && route.back().local==111 && sink.isPinNodeLeased(111),
            "replacement did not use the captured direct arrival and preserve the sink pin");
    require(!sink.cb.dst.jump.testBit(3848) && !sink.cb.joint.jump.testBit(25) &&
            !sink.cb.joint.jump.testBit(11),"replacement leaked the old private Grounding");
    std::printf("private Grounding: retain_old=%d six-hop completion, no preemption, pin retained\n",retain_grounding);
  }
  check_complete_path(f,s.sinks[1].wires.front(),4);
  f.state();
  return repaired;
}

void fanout_private_grounding_regression() {
  require(!run_private_grounding_replay(true,SinkGuard::Shared),"repair rewrote a shared sibling's input");
  require(!run_private_grounding_replay(true,SinkGuard::Alias),"repair rewrote a shared alias's input");
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
void fanout_gap_budget_regression(bool failure_location) {
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
  for (int n=1000;n<1256;++n) {
    f.edge({16,8},{20,20},19,n,n);
    f.name({20,20},fpga::CB_NODE_LOCAL,n);
    f.tile({20,20}).cb_type->dst_local[n].local.setBit(n);
  }
  f.edge({11,6},{12,7},149,906,907,0);
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
  require(!fpga::repairNetRouteGaps(s.net,0,[]{return true;}) && f.state()==before,
          "cancelled repair modified the retained route");
  require(!fpga::repairNetRouteGaps(s.net,0,{},1,200) && f.state()==before,
          "repair bypassed the caller's state limit");
  require(!fpga::repairNetRouteGaps(s.net,0,{},32768,0) && f.state()==before,
          "repair bypassed the caller's hop limit");
  auto &sink_cb=*f.tile({12,7}).cb_type;
  sink_cb.dst_local[907].local.clearBit(107); f.rebuild();
  require(!fpga::repairNetRouteGaps(s.net,0,[&]{return ++polls>200;}) && f.state()==before,
          "unreachable source probe changed routes or their owners");
  sink_cb.dst_local[907].local.setBit(107); f.rebuild();
  std::ostringstream owner_before;
  fpga::dumpNetRouteHistory(f.signal(742).net,owner_before);
  polls=0;
  // A deterministic work deadline, not a flaky wall-clock assertion. The
  // old FIFO gap search consumes this budget before trying the source path.
  const bool repaired=fpga::repairNetRouteGaps(s.net,0,[&]{return ++polls>200;});
  std::printf("gap budget: success=%d cancellation_polls=%u\n",repaired,polls);
  require(repaired,"BUG: detached-gap flood starved an available one-hop source repair");
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
