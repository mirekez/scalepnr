# Routing Rules

This document records generic routing behavior that must stay independent of
any FPGA vendor database.

## Tile Routed Nets

Each tile keeps a list of routed nets that currently use any node of that tile.
The list is an index for local routing decisions; ownership of the concrete
route remains with the sink instance route vector.

When a route is committed, every tile touched by its wire fragments must
reference the route net. When a net is unrouted, all tile references to that net
must be cleared.

## Node Lookup

`findNetByNode(tile, node_type, node, transit_only)` searches only nets
registered on the tile and then scans their route fragments.

With `transit_only = true`, a local-to-exit fragment at route depth 0 is not a
valid victim. This protects a signal that starts from the tile from being
preempted by another signal that also wants to start from the same tile.

## Unrouting

`unrouteNet(net)` removes all dynamic routing ownership for the net:

- source jump leases are cleared from tile crossbar state,
- transit destination leases are cleared from tile crossbar state,
- final tile pin leases are cleared,
- route fragments are removed from the owning route vector,
- tile routed-net references are removed.

Deadend masks are not removed by unrouting. They describe explored bad
directions, not active route ownership.

## Preemption

Takeoff preemption, also called takeof preemption in route debug logs, is
allowed only for a route start at depth 0 and only after the start local cannot
leave the tile through any currently usable direct or joint path.

The preempted route must be a transit route through the tile. A local-to-exit
route is not a legal victim for takeoff preemption, because it is also starting
from the same tile-local resource class.

After preemption, the victim net is queued for rerouting and the starting route
may use the freed exit node.

Grounding preemption is allowed at final tile entry. If a route reaches the
sink tile, the sink pin is free, and the abstract `dst -> local` entry is
possible but the required destination node is occupied by a transit route, the
router may unroute that transit victim, queue the victim net for rerouting, and
let the current route enter the sink. Same-tile final sink entries are not legal
transit victims for this rule; only route fragments that pass through the tile
may be displaced.

## Joint Paths

Joint usage is part of the wire fragment metadata. A route that uses a joint can
be found by node lookup, and a free joint path can be used by a new local start
when the exit node is free.

Outgoing source enumeration must include joint-mediated paths. A local or
destination node with `node -> joint -> src` connectivity is routable even when
there is no direct `node -> src` arc. The crossbar outgoing-source index must
therefore be rebuilt from both direct masks and joint masks after loading or
constructing a crossbar type.

## Route Direction

Route tasks are always driver-output to sink-input. Task collection walks sink
input ports and follows each connection back to its driver, so the first
crossbar local in a route must be an output-capable local node.

`local_src` describes locals that may leave a tile through tile-to-tile source
nodes. `dst_local` describes destination locals that may enter a tile. Some
locals are valid output-only or input-output nodes without direct `local_src`
fanout, for example constant or local-only wires. The crossbar type therefore
keeps direction masks for local input and local output usage. `routeNet` asserts
only if the primary source pin mapping selects an input-only local; fallback
source enumeration skips input-only locals.

The final segment into an output buffer is still a sink-input route. An OBUF
input pin is handled as the route destination and must be leased through the
destination local path, not used as the route source.

## Route Stages

Routing is split into three stages.

Basic routing builds the first route for each driver output port. Task
collection marks every source port it has already emitted and defers later
fanouts from the same source port. This prevents many sinks of one source from
all trying to start at the same source tile and consuming unrelated exits before
a trunk exists. Each pass uses bounded route search and may keep partial
progress so the next pass can continue from the last committed point.

Fanouts routing runs after Basic routing is empty. Deferred fanout tasks look for
an already complete route of the same net, choose a usable branch point on that
route, and route from the branch point to the remaining sink. Branching may
reuse the already occupied incoming destination node for the same net, but it
still must lease a free outgoing source node. Deferred fanouts are still routed
as independent tasks; duplicate tasks for the same sink route are not emitted.

Moving runs when Fanouts routing cannot reduce the unfinished task count. The
router selects an unfinished sink instance and tries nearby legal tile positions
using the generic placement legality checks. If the instance is moved, all route
bindings that touch that instance are unrouted, their tasks are queued again,
and learned deadend masks for those routes are cleared. The moving stage keeps
per-instance tried placement history to avoid cycling through the same failed
positions. It repeats routing and movement until all affected tasks are routed
or no legal movement remains.

## Direct Resource Routes

Some source and sink pins are connected by tile-local or dedicated resource
fabric rather than by crossbar source nodes. If a source pin has mapped output
nodes but no routable crossbar output candidate, the router may represent the
connection as a direct resource route. The route contains tile-pin fragments for
the source marker and final sink pin; only the final sink pin is leased.

This rule is generic: it depends only on abstract source/sink pin mappings and
absence of a routable crossbar output candidate, not on any vendor-specific
resource name.
