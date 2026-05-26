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
    int joint = -1;
    // Resource endpoint metadata annotates tile-pin fragments for export.
    Coord resource;
    int resource_node = -1;
    int pin_dir = -1;
    std::string cell_type;
    std::string port;
    std::string net_name;
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
rtl::Net* findNetByNode(Tile& tile, CBNodeNameType node_type, int node, bool transit_only = false);
bool unrouteNet(rtl::Net& net);
bool unrouteNetRoute(rtl::Net& net, size_t route_binding_index);

}
