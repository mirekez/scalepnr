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

namespace {
void require(bool value, const std::string &message) {
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

  explicit SourceInputFixture(bool output_conflict,int current_wire=0) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    const int height=current_wire==119?16:current_wire==80?7:3;
    device.size_width=8; device.size_height=height; device.grid_spec.size={8,height};
    device.tile_grid.resize(8*height); device.tile_types.reserve(8*height); device.cb_types.reserve(8*height);
    for(int y=0;y<height;++y) for(int x=0;x<8;++x) {
      auto &t=tile({x,y}); auto &cb=device.cb_types.emplace_back();
      cb.name="matrix_"+std::to_string(y*8+x); cb.type_id=cb.base_type_id=y*8+x;
      auto &type=device.tile_types.emplace_back();
      type.name="resource_"+std::to_string(y*8+x); type.type=fpga::Tile::TILE_LUTS;
      t.coord=t.cb_coord={x,y}; t.name=t.coord; t.full_name="tile_"+std::to_string(y*8+x);
      t.type=fpga::Tile::TILE_LUTS; t.tile_type=&type; t.cb_type=t.cb.type=&cb;
      for(int n:{63,87,141,169}) name(t,fpga::CB_NODE_LOCAL,n);
      cb.local_input_nodes.setBit(63); cb.local_input_nodes.setBit(87);
      cb.local_output_nodes.setBit(141); cb.local_output_nodes.setBit(169);
      type.pin_map.nodes[{"FDRE","CE",0}].setBit(63);
      type.pin_map.nodes[{"LUT6","I0",3}].setBit(87);
      type.pin_map.output_nodes[t.getNodeNum("FDRE","Q",0)].setBit(141);
      type.pin_map.output_nodes[t.getNodeNum("LUT6","O",3)].setBit(169);
    }
    for(fpga::Coord c:std::vector<fpga::Coord>{{1,0},{2,0},{7,0}}) {
      fpga::Element e; e.name="register_slot"; e.type=fpga::ELEMENT_FD;
      tile(c).tile_type->elements.push_back(e);
    }
    for(fpga::Coord c:std::vector<fpga::Coord>{{0,0},{6,0}}) {
      fpga::Element e; e.name="logic_slot"; e.type=fpga::ELEMENT_LUT5;
      tile(c).tile_type->elements.push_back(e);
      e.name="logic_extension"; e.type=fpga::ELEMENT_LUT1; e.bitmap_pos=0;
      tile(c).tile_type->elements.push_back(e);
    }
    tech.init(); tech.design.GND=nullptr; tech.design.VCC=nullptr;
    parent=&tech.design.modules.emplace_back(); parent->name="candidate_top";
    parent->nets.reserve(4);
    primitive=&tech.design.modules.emplace_back(); primitive->name="primitive";
    primitive->is_blackbox=true; primitive->parent_ref.set(parent);
    tech.design.top_cell.name=tech.design.top_cell.type="top";
    tech.design.top_cell.module_ref.set(parent);
    tech.design.top.cell_ref.set(&tech.design.top_cell);
    source=cell("register_17","FDRE",{{"CE",false},{"Q",true}},{7,0});
    sink=cell("receiver_19","LUT6",{{"I0",false}},{0,0});
    control=cell("control_23","LUT6",{{"O",true}},{6,0});
    signal=connect(*source,"Q",*sink,"I0"); enable=connect(*control,"O",*source,"CE");
    sink->outline.fixed=control->outline.fixed=true;

    for(int n=0;n<32;++n) {
      edge({2,0},{0,0},141,100+n,500+n,true);
      terminal({0,0},500+n,87);
    }
    edge({1,0},{0,0},141,300,800,true); terminal({0,0},800,87);
    edge({6,0},{1,0},169,900,700,true); terminal({1,0},700,63);

    // Retain eight genuine prefix landings for the control input, matching the
    // failed proof's tree shape. The private old terminal must be replaced.
    std::vector<fpga::Coord> path{{6,0},{6,1},{5,1},{4,1},{3,1},{2,1},{1,1},{0,1},{7,0}};
    // The resolved-link case needs a real occupied ordinary input approach,
    // alongside its free dedicated approach. Detour this existing owner
    // through that CB instead of manufacturing a busy bit without a route.
    if(current_wire==119) path.insert(path.end()-1,{0,0});
    std::vector<fpga::Wire> route{pin(path.front(),169,true)};
    for(size_t n=0;n+1<path.size();++n) {
      const int input=n==0?169:710+int(n)-1;
      route.push_back(edge(path[n],path[n+1],input,910+int(n),710+int(n),n==0));
    }
    route.push_back(terminal({7,0},710+int(path.size())-2,63)); route.push_back(pin({7,0},63,false));

    edge({3,0},{2,0},640,700,640); terminal({2,0},640,63);
    if(output_conflict) {
      // The unrestricted CE route exists but uses the first proposed Q exit.
      // Reject that output only; a later Q exit must still place the register
      // here. A placement-only cache of the original failed proof is unsafe.
      tile({2,0}).cb_type->dst_local[640].local.clearBit(63);
      tile({2,0}).cb_type->dst_src[640].jump.setBit(100);
      edge({6,0},{2,0},169,901,640,true);
      edge({0,0},{2,0},500,750,641); terminal({2,0},641,63);
    } else {
      // This complete free fan-in is disconnected from the control driver and
      // its retained tree. No choice of Q exit can make CE routable here.
      for(int n=0;n<24;++n) {
        tile({3,0}).cb_type->dst_src[650+n].jump.setBit(700);
        edge({4,0},{3,0},1400+n,1000+n,650+n);
      }
    }
    if(current_wire==53) capturedWire();
    if(current_wire==80) capturedWideWire();
    if(current_wire==119) capturedResolvedLocalWire();
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
    require(pnr::commitPreparedRoute(route),"cannot install the retained control tree");
    source->wires.push_back(std::move(route));
    const size_t index=fpga::attachNetRoute(*enable,*source,0,control,source,"O","CE","control_tree");
    fpga::registerNetRouteTiles(*enable,source->wires[0],index);
    tech.route.indexSourceRoute(enable,control,"O");
    audit();
  }

  void capturedWideWire();
  void capturedResolvedLocalWire();

  void resolvedLocal(fpga::Coord from,int local,fpga::Coord to,fpga::CBNodeNameType kind,int node) {
    name(tile(from),fpga::CB_NODE_LOCAL,local); name(tile(to),kind,node);
    fpga::CBType::ResolvedLocal link;
    link.delta=to-from; link.target_cb_type_id=tile(to).cb_type->type_id;
    link.target_node_type=kind; link.target_nodes.setBit(node);
    tile(from).cb_type->local_by_local[local].push_back(link);
  }

  void capturedWire() {
    // Exact 53-node / 21-jump free witness from the second real source
    // failure, with its internal LOCAL bridges. Only Tile coordinates and
    // node labels are remapped. Ten-hop relocation cannot find this route;
    // ordinary current-position routing can, without disturbing the CE tree.
    const std::array<fpga::Coord,22> locations{{{7,0},{7,2},{6,2},{5,2},{4,2},{3,2},
        {2,2},{1,2},{0,2},{0,1},{1,1},{2,1},{3,1},{4,1},{5,1},{6,1},{7,1},
        {5,0},{4,0},{3,0},{2,0},{0,0}}};
    struct Node {int tile,kind,id;};
    const std::array<Node,53> nodes{{
        {0,0,141},{0,3,64},{1,4,64},{1,3,64},{2,4,64},{2,3,3376},
        {3,4,3376},{3,3,3376},{4,4,3376},{4,0,165},{4,0,184},{4,3,98},
        {5,4,98},{5,3,3075},{6,4,3080},{6,3,3539},{7,4,3539},{7,3,163},
        {8,4,163},{8,3,979},{9,4,979},{9,0,140},{9,3,3536},{10,4,3536},
        {10,0,183},{10,0,140},{10,3,3376},{11,4,3376},{11,0,183},{11,3,3075},
        {12,4,1031},{12,3,262},{13,4,260},{13,0,183},{13,3,1031},{14,4,1031},
        {14,3,3840},{15,4,3845},{15,0,178},{15,0,165},{15,3,3078},{16,4,3079},
        {16,3,3810},{17,4,738},{17,3,3078},{18,4,1030},{18,3,194},{19,4,194},
        {19,3,162},{20,4,162},{20,3,3843},{21,4,3848},{21,0,118}}};
    tile({0,0}).tile_type->pin_map.nodes[{"LUT6","I0",3}]={};
    tile({0,0}).tile_type->pin_map.nodes[{"LUT6","I0",3}].setBit(118);
    tile({0,0}).cb_type->local_input_nodes.setBit(118);
    for(size_t n=0;n+1<nodes.size();++n) {
      const auto &a=nodes[n], &b=nodes[n+1]; auto &t=tile(locations[a.tile]);
      name(t,static_cast<fpga::CBNodeNameType>(a.kind),a.id);
      name(tile(locations[b.tile]),static_cast<fpga::CBNodeNameType>(b.kind),b.id);
      if(a.kind==fpga::CB_NODE_SRC && b.kind==fpga::CB_NODE_DST) {
        const auto &before=nodes[n-1];
        edge(locations[a.tile],locations[b.tile],before.id,a.id,b.id,before.kind==fpga::CB_NODE_LOCAL);
      } else if(a.kind==fpga::CB_NODE_DST && b.kind==fpga::CB_NODE_LOCAL) {
        t.cb_type->dst_local[a.id].local.setBit(b.id);
      } else if(a.kind==fpga::CB_NODE_LOCAL && b.kind==fpga::CB_NODE_LOCAL) {
        t.cb_type->local_local[a.id].local.setBit(b.id);
      } else if(b.kind!=fpga::CB_NODE_SRC) {
        require(false,"unexpected captured numeric arc");
      }
    }
  }

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
                 std::initializer_list<std::pair<const char*,bool>> ports,fpga::Coord c) {
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
    const int pos=kind=="LUT6"?3:0;
    require(tile(c).tryAddAt(&i,pos)==pos,"cannot place a fixture endpoint"); return &i;
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
};

void SourceInputFixture::capturedWideWire() {
  // Run 5's failed current-position recovery had this exact 80-node directed
  // path, including its repeated Tile and internal LOCAL bridges. All leases
  // audited clean. Its forward search exhausted 524288 labels; a read-only
  // diagnostic found the free path after 615326 expansions. Preserve the
  // connectivity, remap coordinates/names, and scale the competing free
  // frontier down so the same budget problem is cheap to reproduce.
  std::vector<fpga::Coord> locations{{7,0}};
  for(int x=3;x<=5;++x) locations.push_back({x,0});
  for(int y=2;y<=5;++y) for(int x=0;x<8;++x) locations.push_back({x,y});
  locations.resize(35); locations.push_back({0,0});
  struct Node {int tile,kind,id;};
  const std::array<Node,80> nodes{{
      {0,0,146},{0,3,97},{1,4,97},{1,3,1027},{2,4,1029},{2,3,65},
      {3,4,65},{3,3,817},{4,4,817},{4,3,97},{5,4,97},{5,3,3377},
      {6,4,3377},{6,3,3377},{7,4,3377},{7,3,3377},{8,4,3377},{8,3,3377},
      {9,4,3377},{9,3,3078},{10,4,3079},{10,3,3078},{11,4,3079},{11,3,3377},
      {12,4,3377},{12,3,3538},{13,4,3538},{13,3,3843},{14,4,3848},{14,3,3811},
      {15,4,3811},{15,3,3539},{16,4,3539},{16,3,3539},{17,4,3539},{17,3,3539},
      {18,4,3539},{18,3,163},{19,4,163},{19,3,3077},{20,4,3074},{20,3,979},
      {21,4,979},{21,3,979},{22,4,979},{22,3,3840},{23,4,3845},{23,0,183},
      {23,3,3075},{24,4,1031},{24,3,244},{25,4,248},{25,3,3840},{26,4,260},
      {26,0,196},{26,3,3075},{27,4,1031},{27,0,128},{27,3,3075},{26,4,1031},
      {26,3,262},{28,4,260},{28,0,183},{28,3,3539},{29,4,979},{29,3,1031},
      {30,4,1031},{30,0,128},{30,0,165},{30,3,978},{31,4,978},{31,3,738},
      {32,4,738},{32,3,162},{33,4,162},{33,3,1029},{34,4,1030},{34,3,66},
      {35,4,66},{35,0,102}}};
  tile({7,0}).tile_type->pin_map.output_nodes[tile({7,0}).getNodeNum("FDRE","Q",0)]={};
  tile({7,0}).tile_type->pin_map.output_nodes[tile({7,0}).getNodeNum("FDRE","Q",0)].setBit(146);
  tile({0,0}).tile_type->pin_map.nodes[{"LUT6","I0",3}]={};
  tile({0,0}).tile_type->pin_map.nodes[{"LUT6","I0",3}].setBit(102);
  for(size_t n=0;n+1<nodes.size();++n) {
    const auto &a=nodes[n], &b=nodes[n+1]; auto &t=tile(locations[a.tile]);
    name(t,static_cast<fpga::CBNodeNameType>(a.kind),a.id);
    name(tile(locations[b.tile]),static_cast<fpga::CBNodeNameType>(b.kind),b.id);
    if(a.kind==fpga::CB_NODE_SRC && b.kind==fpga::CB_NODE_DST) {
      // The selected SRC can follow DST or LOCAL; keep that exact distinction.
      const auto &before=nodes[n-1];
      edge(locations[a.tile],locations[b.tile],before.id,a.id,b.id,before.kind==fpga::CB_NODE_LOCAL);
    } else if(a.kind==fpga::CB_NODE_DST && b.kind==fpga::CB_NODE_LOCAL) {
      t.cb_type->dst_local[a.id].local.setBit(b.id);
    } else if(a.kind==fpga::CB_NODE_LOCAL && b.kind==fpga::CB_NODE_LOCAL) {
      t.cb_type->local_local[a.id].local.setBit(b.id);
    } else require(b.kind==fpga::CB_NODE_SRC,"unexpected wide captured arc");
  }
  // A wide free branch cannot reach the sink. A source-only search fills its
  // bounded labels here before traversing the long useful route. No fake busy
  // bits or invalid owners are used to manufacture the congestion.
  edge({7,0},{0,6},146,1,2000,true);
  for(int n=0;n<512;++n) edge({0,6},{1,6},2000,1000+n,2000+n);
}

void SourceInputFixture::capturedResolvedLocalWire() {
  // The final real failure still had this FREE 119-node / 96-Tile-hop path.
  // Its 86 cross-Tile LOCAL links are loader-resolved physical transitions,
  // not CB.local_local arcs. The first bidirectional regression accidentally
  // represented every LOCAL link with that intra-CB table, hiding this gap.
  // Preserve node kinds, adjacency, the same-Tile jump and JOINT pair. Remap
  // coordinates and use abstract labels, then scale the competing frontier.
  std::vector<fpga::Coord> locations{{7,0}};
  for(int x=3;x<=5;++x) locations.push_back({x,0});
  for(int y=2;y<15;++y) for(int x=0;x<8;++x) locations.push_back({x,y});
  locations.resize(96); locations.push_back({0,0});
  struct Node {int tile,kind,id;};
  const std::array<Node,119> nodes{{
      {0,0,145},{0,3,1025},{1,4,1028},{1,3,96},{2,4,96},{2,3,544},
      {3,4,544},{3,3,1024},{4,4,1024},{4,3,816},{5,4,816},{5,3,816},
      {6,4,816},{6,3,1025},{7,4,1028},{7,3,976},{8,4,976},{8,3,3536},
      {9,4,3536},{9,3,160},{10,4,160},{10,3,245},{10,4,240},{10,1,62},
      {10,1,47},{10,0,30},{11,0,80},{12,0,360},{12,0,296},{12,0,345},
      {12,0,281},{12,0,121},{13,0,9},{14,0,37},{15,0,1},{16,0,9},
      {17,0,9},{18,0,9},{19,0,37},{20,0,1},{21,0,9},{22,0,9},
      {23,0,37},{24,0,1},{25,0,9},{26,0,9},{27,0,37},{28,0,1},
      {29,0,9},{30,0,9},{31,0,9},{32,0,37},{33,0,1},{34,0,9},
      {35,0,9},{36,0,37},{37,0,1},{38,0,9},{39,0,9},{40,0,37},
      {41,0,1},{42,0,9},{43,0,9},{44,0,9},{45,0,37},{46,0,1},
      {47,0,9},{48,0,9},{49,0,37},{50,0,1},{51,0,9},{52,0,9},
      {53,0,37},{54,0,1},{55,0,9},{56,0,9},{57,0,9},{58,0,37},
      {59,0,1},{60,0,9},{61,0,9},{62,0,9},{63,0,9},{64,0,37},
      {65,0,1},{66,0,9},{67,0,9},{68,0,37},{69,0,1},{70,0,9},
      {71,0,9},{72,0,9},{73,0,9},{74,0,37},{75,0,1},{75,0,31},
      {76,0,72},{77,0,72},{78,0,72},{79,0,72},{80,0,72},{81,0,72},
      {82,0,72},{83,0,72},{84,0,72},{85,0,72},{86,0,72},{87,0,72},
      {88,0,72},{89,0,72},{90,0,72},{91,0,72},{92,0,72},{93,0,72},
      {94,0,72},{95,0,72},{96,0,72},{96,0,78},{96,0,87}}};
  auto &source_cb=tile({7,0});
  source_cb.tile_type->pin_map.output_nodes[source_cb.getNodeNum("FDRE","Q",0)]={};
  source_cb.tile_type->pin_map.output_nodes[source_cb.getNodeNum("FDRE","Q",0)].setBit(145);
  source_cb.cb_type->local_output_nodes.setBit(145);
  // Drop unrelated free fan-ins from the relocation fixture. Keep the real
  // control transit as an occupied ordinary approach; this signal's free
  // path must enter through the resolved LOCAL chain.
  tile({0,0}).cb_type->dst_local.clear();
  tile({0,0}).cb_type->dst_local[717].local.setBit(87);
  for(size_t n=0;n+1<nodes.size();++n) {
    const auto &a=nodes[n], &b=nodes[n+1]; auto &t=tile(locations[a.tile]);
    name(t,static_cast<fpga::CBNodeNameType>(a.kind),a.id);
    name(tile(locations[b.tile]),static_cast<fpga::CBNodeNameType>(b.kind),b.id);
    if(a.kind==fpga::CB_NODE_SRC && b.kind==fpga::CB_NODE_DST) {
      const auto &before=nodes[n-1];
      require(before.kind==fpga::CB_NODE_LOCAL || before.kind==fpga::CB_NODE_DST,
          "unexpected captured jump input");
      edge(locations[a.tile],locations[b.tile],before.id,a.id,b.id,before.kind==fpga::CB_NODE_LOCAL);
    } else if(a.kind==fpga::CB_NODE_DST && b.kind==fpga::CB_NODE_JOINT) {
      t.cb_type->dst_joint[a.id].joint.setBit(b.id);
    } else if(a.kind==fpga::CB_NODE_JOINT && b.kind==fpga::CB_NODE_JOINT) {
      t.cb_type->joint_joint[a.id].joint.setBit(b.id);
    } else if(a.kind==fpga::CB_NODE_JOINT && b.kind==fpga::CB_NODE_LOCAL) {
      t.cb_type->joint_local[a.id].local.setBit(b.id);
    } else if(a.kind==fpga::CB_NODE_LOCAL && b.kind==fpga::CB_NODE_LOCAL) {
      resolvedLocal(locations[a.tile],a.id,locations[b.tile],fpga::CB_NODE_LOCAL,b.id);
    } else require(b.kind==fpga::CB_NODE_SRC,"unexpected captured resolved-local arc");
  }
  edge({7,0},{0,15},145,1,2000,true);
  for(int n=0;n<512;++n) edge({0,15},{1,15},2000,1000+n,2000+n);
  // A resource view shares another Tile's CB but its dedicated links still
  // start at its own grid coordinate. Equal ids of different target kinds
  // must stay distinct in the reverse index.
  auto &view=tile({7,15});
  view.cb_coord={6,15}; view.cb_type=view.cb.type=tile({6,15}).cb_type;
  resolvedLocal({7,15},3400,{6,15},fpga::CB_NODE_SRC,3330);
  resolvedLocal({7,15},3400,{6,15},fpga::CB_NODE_JOINT,3330);
}
}

void moving_source_candidate_input_work_regression(bool output_conflict) {
  SourceInputFixture f(output_conflict);
  auto &router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=true;
  router.route_preemption_enabled=false; router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  std::vector<std::array<NodeMask,5>> before;
  for(auto &t:f.device.tile_grid)
    before.push_back({t.cb.src.jump,t.cb.dst.jump,t.cb.joint.jump,t.cb.local.local,t.pin_state.leased_nodes});
  auto source_probe=[&](fpga::Tile &tile,int src,pnr::BackwardTakeoffChoice &choice) {
    return router.routeInputSourceTry(*f.control,"O",tile,src,choice);
  };
  auto index=router.backwardDockingIndex({2,0},16);
  require(pnr::inputMayReachSource(f.tile({2,0}),NodeMask{0,1}<<63,{6,0},16,
              source_probe,&index,{},nullptr,nullptr,10)==output_conflict,
      "read-only input possibility check disagrees with the generated graph");
  require(!pnr::inputMayReachSource(f.tile({2,0}),NodeMask{0,1}<<63,{6,0},16,
              source_probe,&index,[]{return true;},nullptr,nullptr,10),
      "cancelled possibility check continued searching");
  for(size_t n=0;n<before.size();++n) {
    auto &t=f.device.tile_grid[n];
    require(before[n]==std::array<NodeMask,5>{t.cb.src.jump,t.cb.dst.jump,t.cb.joint.jump,t.cb.local.local,t.pin_state.leased_nodes},
        "possibility/cancellation check changed live leases");
  }
  pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","candidate_signal"};
  task.endpoints_prepared=true;
  std::ostringstream trace;
  bool moved=false; std::string reason;
  {
    pnr::RouteCongestionTrace capture(task.net_name,"source_input_regression",{f.signal,f.enable},&trace);
    require(router.moveUnfinishedSource(task,nullptr,&reason,nullptr,&moved),
        "source with a proven complete placement failed: "+reason);
  }
  require(moved && f.source->tile.peer==&f.tile({output_conflict?2:1,0}),
      "source did not select the expected complete placement");
  auto complete=[&](rtl::Net &net,rtl::Inst &sink) {
    for(auto &binding:net.routes)
      if(binding.to==&sink && binding.owner && binding.route_index<binding.owner->wires.size() &&
          fpga::isRouteComplete(binding.owner->wires[binding.route_index])) return true;
    return false;
  };
  require(complete(*f.signal,*f.sink) && complete(*f.enable,*f.source),
      "source move did not complete both output and control input");
  require(f.control->tile.peer==&f.tile({6,0}) && f.sink->tile.peer==&f.tile({0,0}),
      "source move relocated a fixed peer");
  f.audit();
  size_t walks=0;
  std::istringstream lines(trace.str()); std::string line;
  while(std::getline(lines,line))
    if(line.find("BACKWARD_BEGIN target=(2,0)")!=std::string::npos) ++walks;
  std::cout << "source input output_conflict=" << output_conflict << " rejected_placement_walks=" << walks << '\n';
  if(!output_conflict)
    require(walks<=2,"source repeated an unreachable control-input search for every output exit");
}

void moving_source_current_wire_regression() {
  SourceInputFixture f(false,53);
  auto &router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=true;
  router.route_preemption_enabled=false; router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  rtl::NetRouteBinding endpoints;
  endpoints.from=f.source; endpoints.to=f.sink; endpoints.from_port="Q"; endpoints.to_port="I0";
  endpoints.route_name="current_wire_53";
  const auto from=SourceInputFixture::pin({7,0},141,true), to=SourceInputFixture::pin({0,0},118,false);
  std::ostringstream witness, control_before;
  fpga::dumpNetRouteHistory(*f.enable,control_before);
  require(fpga::auditRouteCompletion(*f.signal,endpoints,from,to,witness,{},32768,32768,200) &&
      witness.str().find("SOURCE_DIAGNOSTIC_COMPLETE nodes=53 cuts=0")!=std::string::npos,
      "captured 53-node free route was not reproduced exactly");
  require(!fpga::routeUnboundNetFromSource(*f.signal,endpoints,from,to,{},32768,10) &&
      f.sink->wires.empty(),"short relocation bound unexpectedly finished the 21-jump route");
  f.audit();
  pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0",endpoints.route_name};
  task.endpoints_prepared=true;
  bool moved=false; std::string reason;
  const bool done=router.moveUnfinishedSource(task,nullptr,&reason,nullptr,&moved);
  std::cout << "source captured current wire complete=" << done << " moved=" << moved << " reason=" << reason << '\n';
  require(done && !moved && f.source->tile.peer==&f.tile({7,0}) && f.source->pos==0,
      "source relocation search ignored the complete 53-node current-position route");
  require(f.sink->wires.size()==1 && fpga::isRouteComplete(f.sink->wires[0]),
      "current-position source recovery left an unfinished output");
  std::ostringstream control_after; fpga::dumpNetRouteHistory(*f.enable,control_after);
  require(control_before.str()==control_after.str(),"wire-only source recovery rebuilt its control input");
  f.audit();
}

void moving_source_two_ended_regression() {
  {
    SourceInputFixture direct(false);
    direct.edge({7,0},{0,0},141,3300,3301,true);
    direct.terminal({0,0},3301,87);
    // This extra topology was added after the fixture's load-time finalize.
    // Rebuild its derived masks before resolving physical jump landings.
    for(auto &cb:direct.device.cb_types) cb.rebuildOutgoingSrcs();
    direct.device.rebuildIncomingDstMasks();
    rtl::NetRouteBinding endpoints;
    endpoints.from=direct.source; endpoints.to=direct.sink;
    endpoints.from_port="Q"; endpoints.to_port="I0"; endpoints.route_name="exact_budget_wire";
    const auto from=SourceInputFixture::pin({7,0},141,true), to=SourceInputFixture::pin({0,0},87,false);
    const auto landing=direct.device.resolveJumpTargets(direct.tile({7,0}),3300);
    require(landing.size()==1 && landing[0].tile==&direct.tile({0,0}) && landing[0].dst_node==3301,
        "exact-budget fixture did not materialize its physical landing");
    auto index=direct.tech.route.backwardDockingIndex({0,0},8);
    fpga::RepairIncomingSources incoming=[&](fpga::Coord at,fpga::CBNodeNameType type,int node,
        const fpga::RepairIncomingSourceVisitor &visit) {
      pnr::visitBackwardPhysicalSources(index,at,type,node,visit);
    };
    require(!fpga::routeUnboundNetFromSource(*direct.signal,endpoints,from,to,{},2,200,{},incoming) &&
        direct.sink->wires.empty(),"two-ended search exceeded its combined state budget");
    require(!fpga::routeUnboundNetFromSource(*direct.signal,endpoints,from,to,{},4,0,{},incoming) &&
        direct.sink->wires.empty(),"two-ended search exceeded its zero-hop bound");
    direct.audit();
    require(fpga::routeUnboundNetFromSource(*direct.signal,endpoints,from,to,{},4,1,{},incoming),
        "meeting on the final edge unnecessarily consumed a duplicate label");
    direct.audit();
  }
  SourceInputFixture f(false,80);
  rtl::NetRouteBinding endpoints;
  endpoints.from=f.source; endpoints.to=f.sink; endpoints.from_port="Q"; endpoints.to_port="I0";
  endpoints.route_name="current_wire_80";
  const auto from=SourceInputFixture::pin({7,0},146,true), to=SourceInputFixture::pin({0,0},102,false);
  std::ostringstream witness;
  require(fpga::auditRouteCompletion(*f.signal,endpoints,from,to,witness,{},4096,4096,200) &&
      witness.str().find("SOURCE_DIAGNOSTIC_COMPLETE nodes=80 cuts=0")!=std::string::npos,
      "captured 80-node free path was not reproduced");
  f.audit();
  auto index=f.tech.route.backwardDockingIndex({0,0},8);
  fpga::RepairIncomingSources incoming=[&](fpga::Coord at,fpga::CBNodeNameType type,int node,
      const fpga::RepairIncomingSourceVisitor &visit) {
    pnr::visitBackwardPhysicalSources(index,at,type,node,visit);
  };
  require(!fpga::routeUnboundNetFromSource(*f.signal,endpoints,from,to,[]{return true;},256,200,{},incoming) &&
      f.sink->wires.empty(),"cancelled wire search installed a route");
  require(!fpga::routeUnboundNetFromSource(*f.signal,endpoints,from,to,{},256,10,{},incoming) &&
      f.sink->wires.empty(),"two-ended search ignored its hop limit");
  unsigned checks=0;
  require(!fpga::routeUnboundNetFromSource(*f.signal,endpoints,from,to,[&]{return ++checks>12;},256,200,{},incoming) &&
      checks>12 && f.sink->wires.empty(),"mid-search cancellation installed part of the long route");
  f.audit();
  std::ostringstream control_before; fpga::dumpNetRouteHistory(*f.enable,control_before);
  const bool done=fpga::routeUnboundNetFromSource(*f.signal,endpoints,from,to,{},256,200,{},incoming);
  std::cout << "captured wide wire complete=" << done << " state_budget=256\n";
  require(done,"one-ended wire search exhausted its labels in the competing free frontier");
  require(f.sink->wires.size()==1 && fpga::isRouteComplete(f.sink->wires[0]),
      "two-ended search installed an incomplete route");
  std::ostringstream control_after; fpga::dumpNetRouteHistory(*f.enable,control_after);
  require(control_before.str()==control_after.str(),"two-ended search rebuilt the source's control tree");
  f.audit();
}

void moving_source_resolved_local_regression() {
  SourceInputFixture f(false,119);
  rtl::NetRouteBinding endpoints;
  endpoints.from=f.source; endpoints.to=f.sink; endpoints.from_port="Q"; endpoints.to_port="I0";
  endpoints.route_name="current_wire_119";
  const auto from=SourceInputFixture::pin({7,0},145,true), to=SourceInputFixture::pin({0,0},87,false);
  std::ostringstream witness, control_before;
  fpga::dumpNetRouteHistory(*f.enable,control_before);
  require(fpga::auditRouteCompletion(*f.signal,endpoints,from,to,witness,{},4096,4096,200) &&
      witness.str().find("SOURCE_DIAGNOSTIC_COMPLETE nodes=119 cuts=0")!=std::string::npos,
      "captured resolved-local path was not reproduced");
  f.audit();
  auto index=f.tech.route.backwardDockingIndex({0,0},16);
  const auto owners=fpga::findNetOwnersByNode(f.tile({0,0}),fpga::CB_NODE_DST,717);
  require(f.tile({0,0}).cb.dst.jump.testBit(717) && owners.size()==1 && owners.front().net==f.enable,
      "ordinary input obstruction has no real control-route owner");
  require(f.tile({0,0}).cb_type->dsts_reaching_local.get(87).jump.testBit(717) &&
      f.tile({0,0}).incoming_dst_nodes.testBit(717),
      "resolved-local fixture omitted the input's ordinary physical CB approach");
  require(f.device.routeTile(f.tile({7,15}))==&f.tile({6,15}),
      "dedicated-link resource alias was not constructed");
  // Overlapping builds must not duplicate links. Complete lookups must also
  // work through a zero-radius view borrowing the full topology.
  pnr::buildBackwardResolveIndex(f.device,{0,0},16,{},&f.tech.route.docking_resolved_arcs);
  auto narrow=f.tech.route.backwardDockingIndex({0,0},0);
  size_t cross_links=0; bool zero_delta=false, typed_target=false, aliased_source=false;
  for(auto &t:f.device.tile_grid) {
    const auto &cb=*t.cb_type;
    for(const auto &[local,entries]:cb.local_by_local.values) {
      for(const auto &target:f.device.resolveLocalTargets(t,local)) {
        unsigned matches=0;
        pnr::visitBackwardPhysicalSources(narrow,target.tile->coord,target.node_type,target.node,
            [&](fpga::Coord at,fpga::CBNodeNameType type,int node) {
          if(at.x==t.coord.x && at.y==t.coord.y && type==fpga::CB_NODE_LOCAL && node==local) ++matches;
        });
        require(matches==1,"physical LOCAL predecessor missing, mistyped or duplicated");
        if(t.coord.x!=target.tile->coord.x || t.coord.y!=target.tile->coord.y) ++cross_links;
        else zero_delta=true;
        typed_target|=target.node_type!=fpga::CB_NODE_LOCAL;
        aliased_source|=f.device.routeTile(t)!=&t;
      }
    }
  }
  require(cross_links>=86 && zero_delta && typed_target && aliased_source,
      "resolved-local regression lost cross-Tile, zero-delta, typed or resource-view coverage");
  fpga::RepairIncomingSources incoming=[&](fpga::Coord at,fpga::CBNodeNameType type,int node,
      const fpga::RepairIncomingSourceVisitor &visit) {
    pnr::visitBackwardPhysicalSources(index,at,type,node,visit);
  };
  fpga::RepairIncomingSources jumps_only=[&](fpga::Coord at,fpga::CBNodeNameType type,int node,
      const fpga::RepairIncomingSourceVisitor &visit) {
    if(type==fpga::CB_NODE_DST)
      if(const auto *sources=pnr::resolveBackwardSources(index,{at.x,at.y,node},true))
        for(const auto &source:*sources)
          if(source.tile) visit(source.tile->coord,fpga::CB_NODE_SRC,source.src);
  };
  require(!fpga::routeUnboundNetFromSource(*f.signal,endpoints,from,to,{},256,200,{},jumps_only) &&
      f.sink->wires.empty(),"missing-LOCAL negative control unexpectedly completed the captured path");
  require(!fpga::routeUnboundNetFromSource(*f.signal,endpoints,from,to,[]{return true;},256,200,{},incoming) &&
      f.sink->wires.empty(),"cancelled resolved-local search installed a route");
  require(!fpga::routeUnboundNetFromSource(*f.signal,endpoints,from,to,{},256,95,{},incoming) &&
      f.sink->wires.empty(),"resolved-local search ignored its 96-Tile-hop requirement");
  f.audit();
  const bool done=fpga::routeUnboundNetFromSource(*f.signal,endpoints,from,to,{},256,200,{},incoming);
  std::cout << "captured resolved-local wire complete=" << done << " state_budget=256\n";
  require(done,"reverse search omitted physical LOCAL transitions in the 119-node path");
  require(f.sink->wires.size()==1 && fpga::isRouteComplete(f.sink->wires[0]),
      "resolved-local search left an incomplete route");
  std::ostringstream control_after; fpga::dumpNetRouteHistory(*f.enable,control_after);
  require(control_before.str()==control_after.str(),"resolved-local search altered the control tree");
  f.audit();
  const auto binding=f.signal->findRouteBinding(f.source,f.sink,"Q","I0",endpoints.route_name);
  require(binding.index<f.signal->routes.size() && fpga::unrouteNetRoute(*f.signal,binding.index),
      "cannot clear the completed resolved-local route for source-stage replay");
  f.audit();
  auto &router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=true;
  router.route_preemption_enabled=false; router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0",endpoints.route_name};
  task.endpoints_prepared=true;
  bool moved=false; std::string reason;
  const bool replay=router.moveUnfinishedSource(task,nullptr,&reason,nullptr,&moved);
  std::cout << "resolved-local source replay complete=" << replay << " moved=" << moved
      << " pos=" << f.source->pos << " reason=" << reason << '\n';
  require(replay && !moved &&
      f.source->tile.peer==&f.tile({7,0}) && f.source->pos==0,
      "source-stage replay failed to keep its routable source in place: "+reason);
  require(f.sink->wires.size()==1 && fpga::isRouteComplete(f.sink->wires[0]),
      "source-stage replay did not reuse the cleared physical route slot");
  std::ostringstream replay_control; fpga::dumpNetRouteHistory(*f.enable,replay_control);
  require(control_before.str()==replay_control.str(),"source-stage replay altered the control tree");
  f.audit();
}
