#pragma once

#include "Inst.h"
#include "debug.h"
#include "referable.h"
#include "Pin.h"
#include "Crossbar.h"
#include "Net.h"

#include <string>
#include <vector>

namespace fpga {

struct Tile;

struct Wire
{
    enum Type {
      WIRE_CROSSBAR,
      WIRE_TILE_PIN,
    } type = WIRE_CROSSBAR;

    // must have
    Coord from;
    Coord to;

    // optional metadata for local crossbar node to tile resource pin hops
    int local = -1;
    int pos = -1;
    int jump = -1;
    int route_jump = -1;
    int dst = -1;
    int joint = -1;
    // Resource endpoint metadata annotates tile-pin fragments for export.
    Coord resource;
    int resource_node = -1;
    int pin_dir = -1;
    std::string cell_type;
    std::string port;
    std::string net_name;
    // Exact source-side node name for this fragment; transit fragments need this
    // separately from dst_wire_name, which names the landing node on the next tile.
    std::string from_wire_name;
    std::string src_wire_name;
    std::string dst_wire_name;
    // Shared fragments document a reused route-tree trunk for export/readback.
    bool shared = false;

    void assign(rtl::Net* net);
};

void attachNetRoute(rtl::Net& net, rtl::Inst& owner, size_t route_index,
                    rtl::Inst* from, rtl::Inst* to,
                    const std::string& from_port, const std::string& to_port,
                    const std::string& route_name);
void registerNetRouteTiles(rtl::Net& net, const std::vector<Wire>& route);
void registerNetRouteTilesFrom(rtl::Net& net, const std::vector<Wire>& route, size_t first_fragment);
void releaseRouteFragmentLease(const std::vector<Wire>& route, size_t fragment_index);
rtl::Net* findNetByNode(Tile& tile, CBNodeNameType node_type, int node, bool transit_only = false);
bool unrouteNet(rtl::Net& net);
bool unrouteNetBranch(rtl::Net& net, size_t route_binding_index);
bool unrouteBrunch(rtl::Net& net, size_t route_binding_index);
bool discardNetBranch(rtl::Net& net, size_t route_binding_index);
bool unrouteNetRoute(rtl::Net& net, size_t route_binding_index);
bool unrouteNetRouteTree(rtl::Net& net, const std::vector<size_t>& route_binding_indices);

}
