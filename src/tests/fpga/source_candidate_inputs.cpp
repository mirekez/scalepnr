#include "SourceInputFixture.h"

namespace routing_tests {

SourceInputFixture::SourceInputFixture(bool output_conflict,int current_wire,size_t net_capacity,int width,int requested_height) {
    device.tile_grid.clear(); device.tile_types.clear(); device.cb_types.clear();
    device.route_wire_graph.clear(); device.local_route_wire_mappings.clear();
    const int height=requested_height>0?requested_height:current_wire==135?18:current_wire==119?16:current_wire==80?7:3;
    device.size_width=width; device.size_height=height; device.grid_spec.size={width,height};
    device.tile_grid.resize(width*height); device.tile_types.reserve(width*height); device.cb_types.reserve(width*height);
    for(int y=0;y<height;++y) for(int x=0;x<width;++x) {
      auto &t=tile({x,y});t.coord=t.cb_coord={x,y};
      if(requested_height>0 && (x>=8 || y>=3))continue;
      const int type_id=device.cb_types.size();auto &cb=device.cb_types.emplace_back();
      cb.name="matrix_"+std::to_string(y*width+x); cb.type_id=cb.base_type_id=type_id;
      auto &type=device.tile_types.emplace_back();
      type.name="resource_"+std::to_string(y*width+x); type.type=fpga::Tile::TILE_LUTS;
      t.coord=t.cb_coord={x,y}; t.name=t.coord; t.full_name="tile_"+std::to_string(y*width+x);
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
    parent->nets.reserve(net_capacity);
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
    if(current_wire==119 || current_wire==135) path.insert(path.end()-1,{0,0});
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
    if(current_wire==135) capturedDispatcherWire();
    for(auto &cb:device.cb_types) cb.rebuildOutgoingSrcs();
    device.rebuildIncomingDstMasks();
    require(pnr::commitPreparedRoute(route),"cannot install the retained control tree");
    source->wires.push_back(std::move(route));
    const size_t index=fpga::attachNetRoute(*enable,*source,0,control,source,"O","CE","control_tree");
    fpga::registerNetRouteTiles(*enable,source->wires[0],index);
    tech.route.indexSourceRoute(enable,control,"O");
    audit();
  }

void SourceInputFixture::capturedWire() {
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

void SourceInputFixture::capturedDispatcherWire() {
  // Run 5 reached Destination Moving with no binding for this connection.
  // Its audited free completion had 135 nodes and 105 inter-Tile hops; the
  // first two jumps belonged to a complete sibling. Source-only completion
  // exhausted its production bound, although the same physical two-ended
  // search used by Source Moving could join this path. Preserve every node,
  // arc kind and repeated Tile; remap coordinates and abstract names only.
  std::vector<fpga::Coord> locations{{7,0}};
  for(int n=0;n<104;++n) locations.push_back({n%8,2+n/8});
  locations.push_back({0,0});
  struct Node {int tile,kind,id;};
  const std::array<Node,135> nodes{{
      {0,0,141},{0,3,544},{1,4,544},{1,3,3376},{2,4,3376},{2,3,96},
      {3,4,96},{3,3,96},{4,4,96},{4,3,96},{5,4,96},{5,3,96},
      {6,4,96},{6,3,816},{7,4,816},{7,3,816},{8,4,816},{8,3,816},
      {9,4,816},{9,3,96},{10,4,96},{10,3,96},{11,4,96},{11,3,96},
      {12,4,96},{12,3,816},{13,4,816},{13,3,816},{14,4,816},{14,3,816},
      {15,4,816},{15,3,1024},{16,4,1024},{16,0,104},{17,0,72},{18,0,163},
      {18,0,54},{18,0,44},{18,0,92},{19,0,60},{20,0,60},{21,0,60},
      {22,0,60},{23,0,60},{24,0,60},{25,0,60},{26,0,60},{27,0,60},
      {27,0,95},{28,0,60},{29,0,60},{30,0,60},{31,0,60},{32,0,60},
      {33,0,60},{34,0,60},{35,0,416},{36,0,60},{37,0,60},{38,0,60},
      {39,0,60},{40,0,60},{41,0,60},{42,0,60},{43,0,60},{43,0,95},
      {44,0,60},{45,0,60},{46,0,60},{47,0,60},{48,0,60},{49,0,60},
      {50,0,60},{51,0,60},{52,0,60},{53,0,60},{54,0,60},{55,0,60},
      {56,0,28},{57,0,60},{58,0,60},{59,0,60},{60,0,60},{61,0,60},
      {62,0,60},{63,0,60},{64,0,60},{65,0,60},{66,0,60},{67,0,60},
      {68,0,60},{69,0,60},{69,0,95},{70,0,60},{71,0,60},{72,0,60},
      {73,0,60},{74,0,60},{75,0,60},{76,0,60},{77,0,416},{77,0,345},
      {77,0,281},{77,0,121},{78,0,9},{79,0,37},{80,0,1},{81,0,9},
      {82,0,9},{83,0,9},{84,0,37},{85,0,1},{86,0,9},{87,0,9},
      {88,0,37},{89,0,1},{90,0,9},{91,0,9},{92,0,37},{93,0,1},
      {94,0,9},{95,0,9},{96,0,9},{97,0,37},{98,0,1},{98,0,31},
      {99,0,72},{100,0,72},{101,0,72},{102,0,72},{103,0,72},{104,0,72},
      {105,0,72},{105,0,77},{105,0,115}}};
  auto &sink_tile=tile({0,0});
  sink_tile.tile_type->pin_map.nodes[{"LUT6","I0",3}]={};
  sink_tile.tile_type->pin_map.nodes[{"LUT6","I0",3}].setBit(115);
  sink_tile.cb_type->local_input_nodes.setBit(115);
  // The real sink also has ordinary CB fan-in, all occupied. Preserve one
  // such approach with the existing control transit as its real owner; the
  // free completion must use the resolved-LOCAL approach instead.
  sink_tile.cb_type->dst_local[717].local.setBit(115);
  for(size_t n=0;n+1<nodes.size();++n) {
    const auto &a=nodes[n], &b=nodes[n+1]; auto &t=tile(locations[a.tile]);
    name(t,static_cast<fpga::CBNodeNameType>(a.kind),a.id);
    name(tile(locations[b.tile]),static_cast<fpga::CBNodeNameType>(b.kind),b.id);
    if(a.kind==fpga::CB_NODE_SRC && b.kind==fpga::CB_NODE_DST) {
      const auto &before=nodes[n-1];
      edge(locations[a.tile],locations[b.tile],before.id,a.id,b.id,
          before.kind==fpga::CB_NODE_LOCAL);
    } else if(a.kind==fpga::CB_NODE_DST && b.kind==fpga::CB_NODE_LOCAL) {
      t.cb_type->dst_local[a.id].local.setBit(b.id);
    } else if(a.kind==fpga::CB_NODE_LOCAL && b.kind==fpga::CB_NODE_LOCAL) {
      if(a.tile==b.tile) t.cb_type->local_local[a.id].local.setBit(b.id);
      else resolvedLocal(locations[a.tile],a.id,locations[b.tile],fpga::CB_NODE_LOCAL,b.id);
    } else require(b.kind==fpga::CB_NODE_SRC,"unexpected dispatcher witness arc");
  }
  // Both choices start at the occupied, reusable sibling prefix. Scale the
  // competing dead frontier down to exercise the real helper at 256 labels
  // instead of constructing millions of nodes for its production limit.
  edge({1,2},{0,17},3376,3500,2000);
  for(int n=0;n<512;++n) edge({0,17},{1,17},2000,1000+n,2000+n);
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

} // namespace routing_tests

using routing_tests::SourceInputFixture;
using routing_tests::require;

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
    if(line.find("INPUT_PROOF_BEGIN target=(2,0)")!=std::string::npos) ++walks;
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


void moving_destination_unbound_probe_regression() {
  SourceInputFixture f(false,135);
  auto &router=f.tech.route;
  router.complete_routes_only=true;
  router.route_preemption_enabled=false; router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  router.route_recursion_budget=5;
  rtl::Inst sibling;
  std::vector<fpga::Wire> seed{SourceInputFixture::pin({7,0},141,true),
      f.edge({7,0},{0,2},141,544,544,true),
      f.edge({0,2},{1,2},544,3376,3376),
      f.terminal({1,2},3376,600),SourceInputFixture::pin({1,2},600,false)};
  for(auto &cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  require(pnr::commitPreparedRoute(seed),"cannot install the real shared prefix owner");
  sibling.wires.push_back(std::move(seed));
  const size_t shared=fpga::attachNetRoute(*f.signal,sibling,0,f.source,&sibling,"Q","D","retained_seed");
  fpga::registerNetRouteTiles(*f.signal,sibling.wires[0],shared);
  router.indexSourceRoute(f.signal,f.source,"Q");
  f.audit();
  rtl::NetRouteBinding endpoints;
  endpoints.from=f.source; endpoints.to=f.sink; endpoints.from_port="Q"; endpoints.to_port="I0";
  endpoints.route_name="current_wire_135";
  const auto from=SourceInputFixture::pin({7,0},141,true), to=SourceInputFixture::pin({0,0},115,false);
  std::ostringstream witness, control_before, seed_before;
  fpga::dumpNetRouteHistory(*f.enable,control_before);
  fpga::dumpNetRouteHistory(*f.signal,seed_before);
  require(fpga::auditRouteCompletion(*f.signal,endpoints,from,to,witness,{},4096,4096,200) &&
      witness.str().find("SOURCE_DIAGNOSTIC_COMPLETE nodes=135 cuts=0")!=std::string::npos,
      "135-node destination completion witness was not reproduced: "+witness.str());
  pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0",endpoints.route_name};
  task.endpoints_prepared=true;
  // Preserve normal Basic/Fanouts search choices. No failed bounded attempt
  // may publish a binding, change the existing owner, or relocate a cell.
  require(!router.completeTaskFromSource(task,false,256) && f.sink->wires.empty(),
      "source-only negative control unexpectedly completed the competing frontier");
  std::ostringstream seed_after_failure;
  fpga::dumpNetRouteHistory(*f.signal,seed_after_failure);
  require(seed_before.str()==seed_after_failure.str(),"failed completion changed the sibling owner");
  f.audit();
  router.moving_stage=true; router.moving_sources_stage=false;
  const bool done=router.completeTaskFromSource(task,false,256);
  std::cout << "destination physical completion=" << done << " state_budget=256\n";
  require(done,"Destination Moving did not enable its bounded physical two-ended completion");
  require(f.sink->wires.size()==1 && fpga::isRouteComplete(f.sink->wires[0]),
      "destination recovery published an incomplete connection");
  require(f.source->tile.peer==&f.tile({7,0}) && f.sink->tile.peer==&f.tile({0,0}),
      "wire-only completion relocated a cell");
  require(sibling.wires[0].size()==5 && fpga::isRouteComplete(sibling.wires[0]),
      "destination recovery cut its reusable complete sibling");
  std::ostringstream control_after; fpga::dumpNetRouteHistory(*f.enable,control_after);
  require(control_before.str()==control_after.str(),"destination recovery changed the control tree");
  require(router.route_stats.preempt_success==0,"free destination recovery preempted another owner");
  f.audit();
}


void moving_source_generic_relocation_regression() {
  // The real failure had an admitted movable register, an older fixed sink,
  // and a free Takeoff, but no full route from the current register position.
  // Generic recovery failed before invoking Source Moving. Keep those facts
  // with abstract nodes, a live control-input tree and a known legal source
  // placement. A free exit is not proof that a source can stay where it is.
  for(int mode=0;mode<7;++mode) {
    SourceInputFixture f(false);
    auto &router=f.tech.route;
    router.moving_stage=router.moving_sources_stage=true;
    router.route_preemption_enabled=false; router.route_deadends_enabled=false;
    router.iteration_limit=router.route_suffix_depth_limit=200;
    router.preserve_routes=true;
    f.sink->pnr_completed=true;
    f.edge({7,0},{7,2},141,64,64,true); // Free output, disconnected from the sink.
    for(auto &cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","generic_relocation"};
    task.endpoints_prepared=true;
    require(!router.movingSourceNeedsRelocation(task),
        "fixture did not preserve the real free-Takeoff classification");
    router.complete_routes_only=true;
    require(!router.completeTaskFromSource(task) && f.sink->wires.empty(),
        "current placement unexpectedly has a complete wire-only route");
    f.audit();
    if(mode==2) f.source->outline.fixed=f.source->pnr_completed=true;
    if(mode==3) router.moving_sources_stage=false;
    // Destination Moving must also recover a movable source when its load is
    // frozen. The explicit opt-in still keeps nested batches wire-only.
    const bool expect_success=mode<=3 || mode==5;
    bool completed=false, moved=false; std::string reason;
    try {
      // Positive control: production relocation already knows a complete
      // placement, including rerouting the existing control input. Then test
      // that the actual whole-task boundary invokes it before hard failure.
      if(mode>=5) {
        // The no-heartbeat top-level path dispatches a batch. It opts in
        // explicitly; nested input batches retain the wire-only default.
        std::vector<pnr::RouteDesign::RouteTask> tasks{task};
        completed=router.routeTaskBatch(pnr::RouteDesign::RouteTaskMode::Moving,
                                        tasks,1,5,mode==5).completed==1;
      } else {
        completed=mode==0 ? router.moveUnfinishedSource(task,nullptr,&reason,nullptr,&moved)
                          : mode==4 ? router.routeTaskOrFail(task,"Moving sources",5)
                                    : router.routeTaskOrFail(task,"Moving sources",5,true,true);
      }
    } catch(const std::exception &e) {
      if(expect_success) throw;
      reason=e.what();
    }
    if(!expect_success) {
      require(!completed && !reason.empty() && f.source->tile.peer==&f.tile({7,0}) &&
          f.sink->wires.empty(),"whole-task recovery escaped its scope or invented a route");
    } else {
      require(completed && f.source->tile.peer==&f.tile({1,0}) &&
          f.sink->tile.peer==&f.tile({0,0}) && f.sink->outline.fixed && f.sink->pnr_completed,
          "whole-task Source Moving skipped its legal source placement: "+reason);
      require(!f.source->tile->hasFreeElement(fpga::ELEMENT_FD) &&
          f.tile({7,0}).hasFreeElement(fpga::ELEMENT_FD),
          "source relocation did not update packing ownership");
      bool output=false,control=false;
      for(const auto &b:f.signal->routes)
        output|=b.to==f.sink && b.owner && b.route_index<b.owner->wires.size() &&
            fpga::isRouteComplete(b.owner->wires[b.route_index]);
      for(const auto &b:f.enable->routes)
        control|=b.to==f.source && b.owner && b.route_index<b.owner->wires.size() &&
            fpga::isRouteComplete(b.owner->wires[b.route_index]);
      require(output && control,"whole-task relocation left an unfinished output or control input");
    }
    f.audit();
    require(f.tile({7,2}).cb.dst.jump==NodeMask{},"failed Takeoff search leaked its dead landing");
  }
}


// Example 1 finished Basic, then Fanouts left no completed seed for this
// register's output. Destination Moving promoted its unrouted connection to
// Generic. The register could move, its older sink could not, and its current
// free Takeoff led to no complete route. Dispatch incorrectly excluded Source
// recovery solely because the dedicated Source stage had already finished.
// Recreate those roles with neutral node names, a real control-input owner,
// a disconnected current exit, and a known complete replacement placement.
void moving_destination_promoted_source_regression() {
  for(int mode=0;mode<7;++mode) {
    SourceInputFixture f(false);
    auto &router=f.tech.route;
    router.moving_stage=true; router.moving_sources_stage=false;
    router.route_preemption_enabled=false; router.route_deadends_enabled=false;
    router.iteration_limit=router.route_suffix_depth_limit=200;
    router.preserve_routes=true; f.sink->pnr_completed=true;
    f.edge({7,0},{7,2},141,64,64,true);
    for(auto &cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","promoted_signal"};
    task.endpoints_prepared=true; task.fanout=true;
    std::vector<pnr::RouteDesign::RouteTask> tasks{task};
    const auto normalized=pnr::normalizeMovingSourceRoles(tasks,
        [](const auto &) { return std::string("source_173"); },
        [](const auto &) { return false; },[](const auto &) { return false; });
    require(normalized.promoted==1 && !tasks[0].fanout,
        "missing seed did not promote exactly one Generic task");
    task=tasks[0];
    require(!router.movingSourceNeedsRelocation(task),
        "captured source must have a free, disconnected Takeoff");
    router.complete_routes_only=true;
    require(!router.completeTaskFromSource(task) && f.sink->wires.empty(),
        "current source unexpectedly reaches the fixed sink");
    f.audit();
    auto history=[&] { std::ostringstream out;
      fpga::dumpNetRouteHistory(*f.signal,out);fpga::dumpNetRouteHistory(*f.enable,out);
      return out.str(); };
    const auto before=history();
    if(mode==4) router.moving_stage=false; // Outside Moving, even explicit opt-in is inert.
    if(mode==5) router.moving_focus_inst=f.control; // Another active placement owns the move.
    if(mode==6) f.source->outline.fixed=f.source->pnr_completed=true;
    const auto *saved_focus=router.moving_focus_inst;
    const bool expect_success=mode<3 || mode==6;
    bool done=false;std::string reason;
    try {
      if(mode==0) {
        // Independent positive control: the existing source solver can route
        // both Q and CE at the alternative placement before dispatch changes.
        done=router.moveUnfinishedSource(task,nullptr,&reason);
      } else if(mode==2 || mode==3) {
        done=router.routeTaskBatch(pnr::RouteDesign::RouteTaskMode::Moving,
                                  tasks,1,5,mode==2).completed==1;
      } else {
        done=router.routeTaskOrFail(task,"Moving destinations",5,true,true);
      }
    } catch(const std::exception &e) { reason=e.what(); }
    std::cout << "Promoted destination source: mode=" << mode << " complete=" << done
              << " source_stage=" << router.moving_sources_stage << '\n';
    if(expect_success) {
      require(done,"promoted Generic task skipped feasible Source recovery: "+reason);
      require(f.source->tile.peer==&f.tile({1,0}) &&
          f.sink->tile.peer==&f.tile({0,0}) && f.sink->outline.fixed && f.sink->pnr_completed,
          "promoted recovery moved the wrong endpoint");
      require(f.tile({7,0}).hasFreeElement(fpga::ELEMENT_FD) &&
          !f.tile({1,0}).hasFreeElement(fpga::ELEMENT_FD),
          "promoted recovery did not transfer packing ownership");
      for(auto *net:{f.signal,f.enable}) {
        require(!net->routes.empty(),"promoted recovery omitted an incident net");
        for(const auto &b:net->routes)
          require(b.owner && b.route_index<b.owner->wires.size() &&
              fpga::isRouteComplete(b.owner->wires[b.route_index]),
              "promoted recovery left an output or control input incomplete");
      }
    } else {
      require(!done && !reason.empty() && history()==before &&
          f.source->tile.peer==&f.tile({7,0}) && f.sink->wires.empty(),
          "nested/non-Moving/focused guard allowed relocation or leaked routing");
    }
    require(!router.moving_sources_stage && router.moving_focus_inst==saved_focus,
        "promoted source recovery leaked Source-stage or placement focus state");
    require(f.tile({7,2}).cb.dst.jump==NodeMask{},"disconnected source exit leaked its landing");
    f.audit();
  }
}

void moving_source_candidate_input_depth_regression() {
  // The run-5 failure rejected every placement at CE despite finding eight
  // retained control-tree landings. Both placement growth and input repair
  // used the same ten-hop bound. Isolate that hypothesis: Q has a one-hop
  // solution, but CE needs twelve hops from a retained landing. This is a
  // known feasible coupled placement, not a captured bypass in the real run.
  SourceInputFixture f(false);
  auto &router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=true;
  router.route_preemption_enabled=false; router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  // Keep a completed, fixed sibling's tree, rather than misidentifying the
  // moving input's differently named binding as its own retained prefix.
  std::vector<fpga::Wire> sibling_route(f.source->wires.front().begin(),
                                      f.source->wires.front().begin()+8);
  require(fpga::unrouteNetRoute(*f.enable,0),"cannot replace old input owner");
  f.enable->clearRouteBindings();
  fpga::Element sibling_slot; sibling_slot.name="sibling_register";
  sibling_slot.type=fpga::ELEMENT_FD;
  f.tile({3,0}).tile_type->elements.push_back(sibling_slot);
  auto *sibling=f.cell("control_receiver_29","FDRE",{{"CE",false}},{3,0});
  sibling->outline.fixed=true;
  auto &ce=SourceInputFixture::port(*sibling,"CE");
  ce.port_ref->designator=SourceInputFixture::port(*f.source,"CE").port_ref->designator;
  ce.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.control,"O")));
  sibling_route.push_back(f.edge({0,1},{3,0},716,1917,1917));
  sibling_route.push_back(f.terminal({3,0},1917,63));
  sibling_route.push_back(SourceInputFixture::pin({3,0},63,false));
  f.tile({6,0}).cb_type->local_src[169].jump.clearBit(900);
  const std::vector<fpga::Coord> path{{0,1},{0,2},{1,2},{2,2},{3,2},
      {4,2},{5,2},{6,2},{7,2},{5,0},{4,0},{3,0},{1,0}};
  for(size_t n=0;n+1<path.size();++n)
    f.edge(path[n],path[n+1],n?1800+int(n)-1:716,
           1800+int(n),1800+int(n));
  f.terminal(path.back(),1811,63);
  for(auto &cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
  f.device.rebuildIncomingDstMasks();
  require(pnr::commitPreparedRoute(sibling_route),"cannot install completed control sibling");
  sibling->wires.push_back(std::move(sibling_route));
  const size_t sibling_binding=fpga::attachNetRoute(*f.enable,*sibling,0,f.control,sibling,
      "O","CE",ce.makeNetName());
  fpga::registerNetRouteTiles(*f.enable,sibling->wires[0],sibling_binding);
  std::vector<pnr::BackwardRouteAnchor> anchors;
  for(const auto &w:sibling->wires.front())
    if(w.type==fpga::Wire::WIRE_CROSSBAR && w.dst>=0)
      anchors.push_back({&f.tile(w.to),w.dst,w.dst_wire_name,anchors.size()+1});
  require(anchors.size()==8,"fixture lost the eight real control-tree landings");
  auto index=router.backwardDockingIndex({1,0},16);
  auto probe=[&](fpga::Tile &tile,int src,pnr::BackwardTakeoffChoice &choice) {
    return router.routeInputSourceTry(*f.control,"O",tile,src,choice);
  };
  const auto short_proof=pnr::routeBackwardToInput(f.tile({1,0}),NodeMask{0,1}<<63,
      {6,0},16,probe,&index,{},nullptr,&anchors,10);
  const auto complete_proof=pnr::routeBackwardToInput(f.tile({1,0}),NodeMask{0,1}<<63,
      {6,0},16,probe,&index,{},nullptr,&anchors,200);
  require(!short_proof.success && !short_proof.expansion_limit_reached &&
      short_proof.remaining_frontier==0,"ten-hop negative control did not exhaust");
  require(complete_proof.success && complete_proof.completed_from_anchor,
      "known twelve-hop CE continuation is not physically reachable");
  f.audit();
  pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","input_depth"};
  task.endpoints_prepared=true;
  bool moved=false; std::string reason;
  const bool done=router.moveUnfinishedSource(task,nullptr,&reason,nullptr,&moved);
  std::cout << "source input depth short=" << short_proof.success
            << " full=" << complete_proof.success << " moved=" << moved
            << " reason=" << reason << '\n';
  f.audit();
  require(done && moved && f.source->tile.peer==&f.tile({1,0}),
      "short output relocation rejected its feasible longer CE repair: "+reason);
  for(auto *net:{f.enable,f.signal}) {
    require(!net->routes.empty(),"relocation omitted an incident connection");
    for(const auto &binding:net->routes)
      require(binding.owner && binding.route_index<binding.owner->wires.size() &&
          fpga::isRouteComplete(binding.owner->wires[binding.route_index]),
          "relocation published an incomplete input or output");
  }
  require(f.control->tile.peer==&f.tile({6,0}) && f.sink->tile.peer==&f.tile({0,0}),
      "input repair moved a fixed peer");
  f.audit();
}

void moving_fixed_sink_task_relocation_regression() {
  // The 50K run moved a register to repair its input, then hard-failed on its
  // newly invalidated output to a frozen load. The fixed-load recovery helper
  // could relocate the source, but neither whole-task dispatch invoked it.
  // Keep a free but disconnected Takeoff at the first position and a real
  // control-input tree. Only the alternative position reaches the fixed load.
  for (int mode = 0; mode < 5; ++mode) {
    SourceInputFixture f(false);
    auto &router = f.tech.route;
    router.moving_stage = true;
    router.moving_sources_stage = false;
    router.moving_focus_inst = f.source;
    router.route_preemption_enabled = router.route_deadends_enabled = false;
    router.iteration_limit = router.route_suffix_depth_limit = 200;
    router.complete_routes_only = router.preserve_routes = true;
    f.sink->pnr_completed = true;
    f.edge({7,0}, {7,2}, 141, 64, 64, true);
    for (auto &cb : f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
    pnr::RouteDesign::RouteTask task{f.source, f.sink, f.signal, "Q", "I0",
                                    "pattern_moved_output_71"};
    task.endpoints_prepared = true;
    // A source exit exists, but a wire-only retry cannot repair this output.
    require(!router.movingSourceNeedsRelocation(task) &&
            !router.completeTaskFromSource(task),
            "fixture must have a free Takeoff disconnected from the fixed load");
    f.audit();
    if (mode == 3) f.source->outline.fixed = f.source->pnr_completed = true;
    if (mode == 4) {
      router.route_stage_deadline_enabled = true;
      router.route_stage_deadline = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    }
    bool done = false;
    std::string failure;
    try {
      if (mode == 1) {
        std::vector<pnr::RouteDesign::RouteTask> tasks{task};
        done = router.routeTaskBatch(pnr::RouteDesign::RouteTaskMode::Moving,
                                     tasks, 1, 5, true).completed == 1;
      } else {
        done = router.routeTaskOrFail(task, "Moving destinations", 5, true, mode != 2);
      }
    } catch (const std::exception &error) {
      failure = error.what();
    }
    if (mode < 2 || mode == 3) {
      // Both heartbeat and batch dispatch must finish the same fixed-load task,
      // including the moved source's input, before admitting completion.
      require(done && f.source->tile.peer == &f.tile({1,0}),
              "whole-task destination recovery skipped source relocation: " + failure);
      bool output = false, control = false;
      for (const auto &binding : f.signal->routes)
        output |= binding.to == f.sink && binding.owner &&
            binding.route_index < binding.owner->wires.size() &&
            fpga::isRouteComplete(binding.owner->wires[binding.route_index]);
      for (const auto &binding : f.enable->routes)
        control |= binding.to == f.source && binding.owner &&
            binding.route_index < binding.owner->wires.size() &&
            fpga::isRouteComplete(binding.owner->wires[binding.route_index]);
      require(output && control, "relocation left an incomplete incident connection");
      require(f.tile({7,0}).hasFreeElement(fpga::ELEMENT_FD) &&
              !f.tile({1,0}).hasFreeElement(fpga::ELEMENT_FD),
              "relocation did not transfer packing ownership");
    } else {
      // Nested wire-only work and cancellation must not move it.
      require(!done && !failure.empty() && f.source->tile.peer == &f.tile({7,0}),
              "recovery escaped its scope or ignored cancellation");
    }
    // The old chunk remains fixed; neither failed probes nor a successful move
    // may leak a lease on the disconnected output landing.
    require(f.sink->tile.peer == &f.tile({0,0}) && f.sink->outline.fixed &&
            f.sink->pnr_completed && !router.moving_sources_stage &&
            f.tile({7,2}).cb.dst.jump == NodeMask{},
            "recovery changed the frozen sink, stage, or disconnected landing");
    f.audit();
  }
}

void moving_fixed_sink_source_relocation_regression() {
  // Run 5 failed after all trunks had completed: a new movable register's
  // fanout could not reach an older fixed sink. Destination Moving tried
  // wires only and failed without considering source relocation. Reproduce
  // that control flow with a real completed Q sibling and control-input tree.
  // The only feasible current fanout starts after moving the register. Every
  // node is abstract, and every occupied node has a live registered owner.
  for(int mode=0;mode<5;++mode) {
    SourceInputFixture f(false);
    auto &router=f.tech.route;
    router.moving_stage=true; router.moving_sources_stage=false;
    router.route_preemption_enabled=false; router.route_deadends_enabled=false;
    router.iteration_limit=router.route_suffix_depth_limit=200;
    router.complete_routes_only=router.preserve_routes=true;
    f.sink->pnr_completed=true;
    f.enable->routes.front().route_name=SourceInputFixture::port(*f.source,"CE").makeNetName();
    f.enable->invalidateRouteLookup();
    auto &type=*f.tile({7,2}).tile_type;
    fpga::Element e; e.name="logic_slot"; e.type=fpga::ELEMENT_LUT5;
    type.elements.push_back(e); e.name="logic_extension";
    e.type=fpga::ELEMENT_LUT1; e.bitmap_pos=0; type.elements.push_back(e);
    auto *sibling=f.cell("existing_receiver_31","LUT6",{{"I0",false}},{7,2});
    sibling->outline.fixed=sibling->pnr_completed=true;
    auto &sibling_input=SourceInputFixture::port(*sibling,"I0");
    sibling_input.port_ref->designator=SourceInputFixture::port(*f.source,"Q").port_ref->designator;
    sibling_input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.source,"Q")));
    const auto sibling_name=sibling_input.makeNetName();
    std::vector<fpga::Wire> old_route{SourceInputFixture::pin({7,0},141,true),
        f.edge({7,0},{7,2},141,64,64,true),f.terminal({7,2},64,87),
        SourceInputFixture::pin({7,2},87,false)};
    // Once the required fanout reaches its fixed sink, its landing also
    // supplies the replacement branch for the displaced completed sibling.
    f.edge({0,0},{7,2},800,65,65); f.terminal({7,2},65,87);
    for(auto &cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
    require(pnr::commitPreparedRoute(old_route),"cannot install completed Q sibling");
    sibling->wires.push_back(std::move(old_route));
    size_t old_binding=fpga::attachNetRoute(*f.signal,*sibling,0,f.source,sibling,
                                          "Q","I0",sibling_name);
    fpga::registerNetRouteTiles(*f.signal,sibling->wires[0],old_binding);
    router.indexSourceRoute(f.signal,f.source,"Q");
    require(router.sourceTreeHasCompleteExit(*f.source,"Q"),"fixture lost its completed trunk");
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0",
        SourceInputFixture::port(*f.sink,"I0").makeNetName()};
    task.fanout=task.endpoints_prepared=true;
    require(!router.completeTaskFromSource(task),"old placement unexpectedly reaches fixed sink");
    f.audit();
    if(mode==2) f.source->outline.fixed=f.source->pnr_completed=true;
    if(mode==3) {
      router.route_stage_deadline_enabled=true;
      router.route_stage_deadline=std::chrono::steady_clock::now()-std::chrono::seconds(1);
    }
    bool moved=false;std::string reason;
    const bool done=mode==0 ? router.moveUnfinishedSource(task,nullptr,&reason,nullptr,&moved)
                            : router.moveUnfinishedDestination(task,nullptr,&reason,mode!=4);
    f.audit();
    if(mode>=3) {
      require(!done && f.source->tile.peer==&f.tile({7,0}) &&
          fpga::isRouteComplete(sibling->wires[0]),
          "cancelled or nested recovery moved the source or damaged its trunk");
      continue;
    }
    require(done && f.source->tile.peer==&f.tile({1,0}),
        "fixed-sink recovery skipped a proven source relocation: "+reason);
    require(f.sink->tile.peer==&f.tile({0,0}) && sibling->tile.peer==&f.tile({7,2}) &&
        f.sink->outline.fixed && sibling->outline.fixed && !router.moving_sources_stage,
        "recovery moved an old sink or leaked a stage change");
    pnr::RouteDesign::RouteTask replacement{f.source,sibling,f.signal,"Q","I0",sibling_name};
    replacement.fanout=replacement.endpoints_prepared=true;
    require(router.routeTaskOrFail(replacement,"Moving destinations",5),
        "cannot repair sibling displaced by source relocation");
    for(auto *net:{f.enable,f.signal}) for(const auto &b:net->routes)
      if(b.owner && b.route_index<b.owner->wires.size() && !b.owner->wires[b.route_index].empty())
        require(fpga::isRouteComplete(b.owner->wires[b.route_index]),"source fallback left incomplete input/output");
    f.audit();
  }
}

void moving_source_disconnected_control_regression() {
  // The latest example stopped on a movable register whose CE tree had one
  // complete sibling with six fabric landings, all outside the explored Q
  // destination region. Every candidate failed its free CE proof. This is a
  // minimization of that OBSERVED structure, not an imported real-device
  // bypass: the snapshot does not establish that such a bypass exists.
  // A closed case must fail without publishing Q or damaging that sibling;
  // opening a known twelve-hop continuation must make relocation succeed.
  // All synthetic node names are generated by SourceInputFixture::name.
  for(bool open_control : {false,true}) {
    SourceInputFixture f(false);
    auto &router=f.tech.route;
    router.moving_stage=router.moving_sources_stage=true;
    router.route_preemption_enabled=false; router.route_deadends_enabled=false;
    router.iteration_limit=router.route_suffix_depth_limit=200;
    std::vector<fpga::Wire> retained(f.source->wires.front().begin(),
                                   f.source->wires.front().begin()+6);
    require(fpga::unrouteNetRoute(*f.enable,0),"cannot replace original CE owner");
    f.enable->clearRouteBindings();
    fpga::Element slot; slot.name="stationary_control_receiver";
    slot.type=fpga::ELEMENT_FD;
    f.tile({3,0}).tile_type->elements.push_back(slot);
    auto *sibling=f.cell("control_receiver_37","FDRE",{{"CE",false}},{3,0});
    sibling->outline.fixed=sibling->pnr_completed=true;
    auto &input=SourceInputFixture::port(*sibling,"CE");
    input.port_ref->designator=SourceInputFixture::port(*f.source,"CE").port_ref->designator;
    input.set(&rtl::Conn::fromBase(SourceInputFixture::port(*f.control,"O")));
    retained.push_back(f.edge({2,1},{3,0},714,1917,1917));
    retained.push_back(f.terminal({3,0},1917,63));
    retained.push_back(SourceInputFixture::pin({3,0},63,false));
    f.tile({6,0}).cb_type->local_src[169].jump.clearBit(900);
    if(open_control) {
      const std::vector<fpga::Coord> path{{2,1},{0,2},{1,2},{2,2},{3,2},
          {4,2},{5,2},{6,2},{7,2},{5,0},{4,0},{3,0},{1,0}};
      for(size_t n=0;n+1<path.size();++n)
        f.edge(path[n],path[n+1],n?1800+int(n)-1:714,
               1800+int(n),1800+int(n));
      f.terminal(path.back(),1811,63);
    }
    for(auto &cb:f.device.cb_types) cb.rebuildOutgoingSrcs();
    f.device.rebuildIncomingDstMasks();
    require(pnr::commitPreparedRoute(retained),"cannot install completed CE sibling");
    sibling->wires.push_back(std::move(retained));
    const size_t binding=fpga::attachNetRoute(*f.enable,*sibling,0,f.control,sibling,
                                            "O","CE",input.makeNetName());
    fpga::registerNetRouteTiles(*f.enable,sibling->wires[0],binding);
    std::vector<pnr::BackwardRouteAnchor> landings;
    for(const auto &w:sibling->wires.front())
      if(w.type==fpga::Wire::WIRE_CROSSBAR && w.dst>=0)
        landings.push_back({&f.tile(w.to),w.dst,w.dst_wire_name,landings.size()+1});
    require(landings.size()==6,"fixture does not match six retained CE landings");
    auto index=router.backwardDockingIndex({1,0},16);
    auto probe=[&](fpga::Tile &tile,int src,pnr::BackwardTakeoffChoice &choice) {
      return router.routeInputSourceTry(*f.control,"O",tile,src,choice);
    };
    const auto proof=pnr::routeBackwardToInput(f.tile({1,0}),NodeMask{0,1}<<63,
        {6,0},16,probe,&index,{},nullptr,&landings,200);
    require(proof.success==open_control && !proof.expansion_limit_reached &&
        proof.remaining_frontier==0,"control proof disagrees with explicit connectivity");
    f.audit();
    pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0",
        SourceInputFixture::port(*f.sink,"I0").makeNetName()};
    task.endpoints_prepared=true;
    bool moved=false; std::string reason;
    const bool done=router.moveUnfinishedSource(task,nullptr,&reason,nullptr,&moved);
    std::cout << "six-landing CE replay open=" << open_control << " complete=" << done
              << " moved=" << moved << " reason=" << reason << '\n';
    require(done==open_control && moved==open_control,
        "source mover disagrees with the known coupled solution: "+reason);
    require(f.source->tile.peer==&f.tile(open_control?fpga::Coord{1,0}:fpga::Coord{7,0}),
        "source mover published an unexpected placement");
    require(sibling->tile.peer==&f.tile({3,0}) &&
        fpga::isRouteComplete(sibling->wires.front()),"source probing damaged its fixed CE sibling");
    if(!open_control) require(f.sink->wires.empty(),"closed input accepted an output-only solution");
    f.audit();
  }
}

void moving_source_timeout_diagnostics_regression() {
  // No sleep, shortened search budget, or altered clock is needed to check
  // the half-open last-100-ms window and the real source-input diagnostic.
  using namespace std::chrono;
  const auto deadline=steady_clock::time_point{}+seconds(10);
  require(!pnr::sourceTimeoutDiagnosticWindow(true,true,deadline,deadline-milliseconds(101),milliseconds(100)) &&
      pnr::sourceTimeoutDiagnosticWindow(true,true,deadline,deadline-milliseconds(100),milliseconds(100)) &&
      pnr::sourceTimeoutDiagnosticWindow(true,true,deadline,deadline-milliseconds(1),milliseconds(100)) &&
      !pnr::sourceTimeoutDiagnosticWindow(true,true,deadline,deadline,milliseconds(100)) &&
      !pnr::sourceTimeoutDiagnosticWindow(true,true,deadline,deadline+milliseconds(1),milliseconds(100)) &&
      !pnr::sourceTimeoutDiagnosticWindow(false,true,deadline,deadline-milliseconds(1),milliseconds(100)) &&
      !pnr::sourceTimeoutDiagnosticWindow(true,false,deadline,deadline-milliseconds(1),milliseconds(100)) &&
      !pnr::sourceTimeoutDiagnosticWindow(true,true,deadline,deadline-milliseconds(1),milliseconds(0)),
      "late source diagnostics escaped their enabled stage/deadline window");
  // This test is a separate process; set the opt-in before its first router
  // invocation. The larger window avoids waiting and leaves the normal
  // successful fixture enough time to finish while its trace is captured.
  setenv("SCALEPNR_SOURCE_TIMEOUT_TRACE_MS","10000",1);
  SourceInputFixture f(false);
  auto &router=f.tech.route;
  router.moving_stage=router.moving_sources_stage=true;
  router.route_preemption_enabled=false; router.route_deadends_enabled=false;
  router.iteration_limit=router.route_suffix_depth_limit=200;
  router.route_stage_deadline_enabled=true;
  router.route_stage_deadline=steady_clock::now()+seconds(10);
  std::ostringstream out;
  struct Capture {
    std::streambuf *old;
    explicit Capture(std::ostream &out):old(std::cout.rdbuf(out.rdbuf())){}
    ~Capture(){std::cout.rdbuf(old);}
  } capture(out);
  pnr::RouteDesign::RouteTask task{f.source,f.sink,f.signal,"Q","I0","timeout_control_probe"};
  task.endpoints_prepared=true;
  std::string reason;
  require(router.moveUnfinishedSource(task,nullptr,&reason) && f.source->tile.peer==&f.tile({1,0}),
      "diagnostics changed the known coupled relocation: "+reason);
  const auto text=out.str();
  require(text.find("near_source_timeout=1")!=std::string::npos &&
      text.find("SOURCE_CANDIDATE_INPUT_VIEW")!=std::string::npos &&
      text.find("input=\"CE\"")!=std::string::npos &&
      text.find("task_driver_matches=1 task_net_matches=1")!=std::string::npos &&
      text.find("RESOURCE_CONTENTS")!=std::string::npos &&
      text.find("BACKWARD_POP")!=std::string::npos &&
      text.find("SNAPSHOT_BEGIN")!=std::string::npos &&
      text.find("SOURCE_CANDIDATE_INPUT_RESULT")!=std::string::npos &&
      !pnr::RouteCongestionTrace::current && !router.route_stage_deadline_expired,
      "late diagnostic omitted the CE identity, candidate state, decisions, or trace restoration");
  f.audit();
}
