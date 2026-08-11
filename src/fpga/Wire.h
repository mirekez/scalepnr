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

struct NetRouteRef
{
    rtl::Net* net = nullptr;
    size_t binding_index = 0;
};

struct Wire
{
    enum Type {
      WIRE_CROSSBAR,
      WIRE_TILE_PIN,
      // One directed edge between numeric nodes in a loaded routing graph.
      WIRE_ROUTE_EDGE,
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
    // A fork may use its parent trunk destination without owning that lease.
    bool owns_dst = true;
    // A partial route's final jump reserves its destination until continuation.
    bool owns_landing = false;

    void assign(rtl::Net* net);
};

void attachNetRoute(rtl::Net& net, rtl::Inst& owner, size_t route_index,
                    rtl::Inst* from, rtl::Inst* to,
                    const std::string& from_port, const std::string& to_port,
                    const std::string& route_name);
// Move route ownership to replacement physical endpoints while preserving its logical name.
size_t retargetNetRouteBindings(rtl::Net& old_net, rtl::Net& new_net,
                                rtl::Inst* old_from, rtl::Inst* old_to,
                                const std::string& old_from_port, const std::string& old_to_port,
                                rtl::Inst* new_from, rtl::Inst* new_to,
                                const std::string& new_from_port, const std::string& new_to_port,
                                const std::string& route_name);
// Retarget every sibling binding that shares one physical source endpoint.
size_t retargetNetRouteSourceBindings(rtl::Net& net, rtl::Inst* old_from,
                                      const std::string& old_from_port,
                                      rtl::Inst* new_from, const std::string& new_from_port);
void registerNetRouteTiles(rtl::Net& net, const std::vector<Wire>& route);
void registerNetRouteTilesFrom(rtl::Net& net, const std::vector<Wire>& route, size_t first_fragment);
// A completed physical route joins two endpoint pins and crosses fabric when
// those endpoints belong to different tiles.
bool isRouteComplete(const std::vector<Wire>& route);
void releaseRouteFragmentLease(const std::vector<Wire>& route, size_t fragment_index);
// Release a route fragment set using one owner scan per affected tile.
void releaseRouteLeases(const std::vector<Wire>& route);
rtl::Net* findNetByNode(Tile& tile, CBNodeNameType node_type, int node, bool transit_only = false);
// Return every route binding using a physical node so shared/stale ownership is handled atomically.
std::vector<NetRouteRef> findNetRoutesByNode(Tile& tile, CBNodeNameType node_type,
                                             int node, bool transit_only = false);
// Return only bindings that own the node lease; shared route-tree replicas are excluded.
std::vector<NetRouteRef> findNetOwnersByNode(Tile& tile, CBNodeNameType node_type,
                                             int node, bool transit_only = false);
bool unrouteNet(rtl::Net& net);
// Clear only bindings for one exact physical endpoint connection.
// Other designators and fanout trees stored in the same RTL net remain routed.
size_t unrouteNetConnection(rtl::Net& net, rtl::Inst* from, rtl::Inst* to,
                            const std::string& from_port, const std::string& to_port);
bool unrouteNetBranch(rtl::Net& net, size_t route_binding_index);
bool unrouteBrunch(rtl::Net& net, size_t route_binding_index);
bool detachNetRouteDestination(rtl::Net& net, size_t route_binding_index);
bool invalidateMovedSinkRoute(rtl::Net& net, size_t route_binding_index);
// Invalidate co-moved sinks atomically so they cannot preserve each other's
// obsolete shared prefixes.
bool invalidateMovedSinkRoutes(const std::vector<NetRouteRef>& routes);
bool discardNetBranch(rtl::Net& net, size_t route_binding_index);
bool unrouteNetRoute(rtl::Net& net, size_t route_binding_index);
// Remove the suffix beginning at one physical node while retaining the
// committed prefix immediately before that node.
bool unrouteNetRouteFromNode(rtl::Net& net, size_t route_binding_index,
                             Coord tile, CBNodeNameType node_type, int node);
// Retain the source endpoint and first physical takeoff while releasing the
// rest of one incomplete route.
bool unrouteNetRouteToTakeoff(rtl::Net& net, size_t route_binding_index);
// Verify one conflicting transit node, then retain only the source endpoint
// and first physical takeoff while releasing the displaced route remainder.
bool unrouteNetRouteToTakeoffFromNode(rtl::Net& net, size_t route_binding_index,
                                      Coord tile, CBNodeNameType node_type,
                                      int node);
// Release one unique route tail step while retaining the preceding committed prefix.
bool unrouteLastRouteStep(rtl::Net& net, size_t route_binding_index);
bool unrouteNetRouteTree(rtl::Net& net, const std::vector<size_t>& route_binding_indices);
// Release every binding in one complete physical source tree atomically.
bool unrouteSourceRouteTree(const std::vector<NetRouteRef>& routes);

}
