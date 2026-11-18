#pragma once

#include "Inst.h"
#include "debug.h"
#include "referable.h"
#include "Pin.h"
#include "Crossbar.h"
#include "Net.h"
#include "InternedString.h"

#include <functional>
#include <string>
#include <array>
#include <iosfwd>
#include <memory>
#include <string_view>
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace fpga {

// Shared grounding window for forward recovery and backward docking.
inline constexpr int GROUNDING_RADIUS = 5;

struct Tile;

struct Wire
{
    enum Type {
      WIRE_CROSSBAR,
      WIRE_TILE_PIN,
      // One directed edge between numeric nodes in a loaded routing graph.
      WIRE_ROUTE_EDGE,
      // No physical resources. Both sides remain leased, but the source and
      // sink are disconnected until this DST-to-DST (or terminal DST-to-LOCAL)
      // gap is repaired. A LOCAL endpoint stays owned while its arrival changes.
      WIRE_GAP,
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
    // First joint in a two-joint path; joint remains the joint adjacent to src/local.
    int joint2 = -1;
    int from_node_type = -1;
    int from_node = -1;
    int to_node_type = -1;
    int to_node = -1;
    // Resource endpoint metadata annotates tile-pin fragments for export.
    Coord resource;
    int resource_node = -1;
    int pin_dir = -1;
    InternedString cell_type;
    InternedString port;
    InternedString net_name;
    // Exact source-side node name for this fragment; transit fragments need this
    // separately from dst_wire_name, which names the landing node on the next tile.
    InternedString from_wire_name;
    InternedString src_wire_name;
    InternedString dst_wire_name;
    // Shared fragments document a reused route-tree trunk for export/readback.
    bool shared = false;
    // A fork may use its parent trunk destination without owning that lease.
    bool owns_dst = true;
    // A partial route's final jump reserves its destination until continuation.
    bool owns_landing = false;

    void assign(rtl::Net* net);
};

// A completed physical route joins two endpoint pins and crosses fabric when
// those endpoints belong to different tiles.
bool isRouteComplete(const std::vector<Wire>& route);
enum class RouteState { Unrouted, PartiallyRouted, Routed };
RouteState routeState(const std::vector<Wire>& route);
bool hasRouteGaps(const std::vector<Wire>& route);

}
