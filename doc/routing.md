# Routing Rules

This document records generic routing behavior that must stay independent of
any FPGA vendor database.

## Constant-One Routing

Logical constant-one inputs remain attached to the design's global constant
connection. A crossbar database may declare continuously driven local nodes in
`constant_one_nodes`. `RouteVCC` only discovers those logical loads, creates one
stable logical source/net identity, and prepares any required tile-local
passthrough endpoints. It does not search paths, lease nodes, preempt routes, or
run a separate retry loop.

`RouteDesign` turns every prepared load into an ordinary route task. These
independent distributed-source tasks participate in Generic routing and, when a
load cannot use its current placement, Moving routing. They do not participate
in Fanout routing because each target crossbar supplies its own physical root.
Generic first preserves the established ordinary-trunk ordering, then attempts
the mandatory local-only tasks. Basic does not displace a completed ordinary
trunk: a blocked constant load follows the normal escalation into Moving so
placement recovery is tried first. If every numeric path remains occupied in
Moving, the router may select a path only when every foreign owner on it is a
preemptible transit route. A route terminating on a resource in that tile keeps
its endpoint; the distributed task remains unfinished so Moving can relocate
its own packed sink. For transit congestion, the router first releases only the
branch suffix beginning at the exact blocking local or joint and requeues that
endpoint through the normal scheduler. If shared ownership makes the suffix
indivisible, it falls back to releasing the physical source tree atomically and
returns one Generic seed plus its dependent branches. Protected routes and
leased nodes without a live owner are never displaced.
The local path search uses only the numeric `local_local`, `local_joint`,
`joint_joint`, and `joint_local` masks. Runtime routing does not inspect node
names or architecture-specific text.

When a generated constant endpoint feeds a packed multi-column element chain,
placement reserves the predecessor lane selected by the target port. Rehoming
accepts a shared output-local identity only when the loaded element graph proves
transitive strict-chain connectivity between its owners.

The physical net is protected from preemption by unrelated nets because its
local root is a mandatory continuously driven resource. Protection does not
remove it from generic completion, incident-route, or Moving audits. Moving a
constant load must release and requeue its old local branch. A final audit uses
the original prepared task identities, not only surviving route bindings, so a
removed branch cannot silently disappear.

One logical distributed source may produce several disconnected physical
roots. The design-state database writes each connected root as a separate route
tree while preserving the common logical net and source identity. This keeps
route trees structurally connected and lets an architecture-specific exporter
map the generic constant source metadata to its physical static-net source.

## Route Stages

Routing is split into three top-level stages. A stage is the scheduler phase;
passes are the lower-level bounded iterations run inside the currently active
stage. The stages are ordered intentionally: first build one trunk per source,
then add remaining fanouts from existing trunks, then move cells only when
routing cannot converge with the current placement. Each stage has many passes;
the stage owns the timeout and the passes own only bounded search work.

Every physical route is identified by its logical net, driver instance and pin,
sink instance and pin, and route name. A route name by itself is not an identity.
Attaching the same exact endpoint identity updates its incomplete route-vector
owner instead of creating another task. A completed physical binding is never
replaced by a later incomplete attachment.

A route is complete only when its committed fragments reach the required sink
tile pin. Partial crossbar progress is retained as a prefix, but it is not
reported as a completed route binding.

### 1. Generic Routing

Generic routing builds the first route for each driver output port. Task
collection walks sink input ports and follows each connection back to its
driver. It marks each source port after emitting its first task and defers later
fanouts from that same source port. This prevents many sinks of one source from
all trying to start at the same source tile and consuming unrelated exits before
a trunk exists.

The search uses only numeric crossbar masks. At an incoming destination or local
source, it enumerates direct and joint-mediated outgoing source bits. Source
bits are visited by angle relative to the destination; for equal angles, shorter
jumps have priority. Once a source bit is selected, `dst_by_src` resolves its
exact numeric landing node and coordinate delta. Textual wire names are never
used to choose a route at runtime.

One bounded continuation explores at most the configured short recursion depth,
normally five hops. Speculative suffix fragments and their leases remain local
to that search. A successful suffix is appended to the route vector and leased
atomically. A failed suffix returns to its parent so another outgoing source can
be tried. Successful partial progress is committed even when the destination is
not yet reached, allowing the next pass to continue at the committed endpoint.

Generic routing runs bounded passes until every source-port seed is complete.
If a full attempt cannot retain a useful prefix, the task remains scheduled for
another pass. The scheduler must not lose an empty route merely because no lease
was released during its latest invalidation.

Source deadend masks are learned and enforced only during Generic routing. They
record failed bounded trunk continuations and steer later Generic passes away
from the same exits. A failed edge is marked only after its child continuation
has returned without a committed suffix. Docking ignores these persistent marks
because final entry is a different bounded search problem.

Generic routing needs one completed trunk per physical source port. If the
selected seed sink repeatedly fails after its partial source tree is released,
the scheduler returns that sink to Fanout work and tries another deferred sink
from the same source. One difficult endpoint therefore cannot permanently pin
the source's Generic trunk selection.

### 2. Fanout Routing

Fanout routing runs after Generic routing has built the initial source trunks.
Like Generic routing, it advances through bounded passes inside the Fanout
stage. Every Fanout task must have at least one already routed source exit from
Generic routing. A source marker or tile-local endpoint is not enough; Fanout
mode never routes from the source tile.

Fanout routing follows the routed trunk for the same physical source pin and
looks for a branch point. A branch point is a transit destination node already
used by the trunk where the signal can fork through an additional outgoing
source node. The quick branchability rule is generic: count currently available
outgoing fork exits from that transit node, using the same dynamic lease checks
as routing. If more than two exits are available, the router may branch there
and route the current sink from that point.

The branch route copies the shared trunk prefix as shared fragments and owns
only the new private suffix. The already leased trunk destination may therefore
appear in several bindings of the same physical source tree, while each private
outgoing source, joint, destination, and sink local still has one owner. Fanout
completion never creates another source-tile takeoff.

If the trunk has no preferred branch point, the follower records a fallback
point at the end of the trunk. Before using that fallback, Fanout routing checks
already routed sibling routes from the same physical source pin and applies the
same branch search to them. Only after the trunk and routed siblings have no
preferred branch point may fallback points be tried. If no trunk or sibling can
provide any branch point, Fanout routing leaves the task unfinished for the
Moving stage instead of starting a new source-tile route.

Branching may reuse the already occupied incoming destination node for the same
net, but it still must lease a free outgoing source node. Deferred fanouts are
still routed as independent sink tasks; duplicate tasks for the same sink route
are not emitted.

Fanout routing ignores persistent source deadend masks and does not add new
persistent marks. Within one bounded search it still remembers each failed
child edge, returns to that child's parent, and tries another exit. This
temporary rejection is discarded when the search returns. Fanout preemption
changes congestion after Generic routing, so an exit learned as unproductive by
an earlier trunk attempt is not a valid permanent exclusion. Real source,
destination, joint, and local leases remain enforced.
If Fanout preemption requeues a Generic trunk repair, that repair remains part
of the Fanout stage policy and therefore does not re-enable deadend masks.

Removing a transit trunk invalidates the complete physical source tree, not only
the binding that exposed the conflict. The invalidation sequence is atomic:

1. Collect every binding driven by the same source instance and source pin,
   including bindings stored under different logical net objects.
2. Release every surviving lease for those bindings.
3. Requeue every collected binding, including siblings already emptied by an
   earlier cleanup operation.
4. If no completed external seed survives, mark exactly the first restored task
   as Generic and all remaining tasks as Fanout. If a seed still survives, keep
   every restored binding as Fanout.
5. Rebuild the trunk before rebuilding its dependent branches.

Step 3 is mandatory even when physical cleanup reports that it changed no
state. An empty route still represents unfinished scheduler work. Omitting it
causes route bindings to disappear until a final audit, producing large late
Fanout or Moving regressions.

### 3. Moving Routing

Moving routing runs when Fanout routing cannot reduce the unfinished task count.
It is still a stage, and its relocation attempts are followed by bounded Generic
and Fanout passes for only the affected task set. The router selects an
unfinished sink endpoint or strict packing cluster and tries nearby legal tile
positions using the generic placement legality checks. Ordinary load cells are
the relocation targets; a completed driver is not moved merely because one of
its fanouts is blocked.

If a fanout sink is moved, only its private branch suffix is unrouted. The
shared trunk and sibling fanouts retain their routes and leases. Moving keeps
per-instance tried placement history to avoid cycling through the same failed
positions.

All input suffixes incident to the moved sink are invalidated because every old
sink local belongs to the previous placement. Output source trees are invalidated
only when the moved packing cluster actually contains their physical driver.
Generated passthrough elements connected by void resource nets move with their
owning cluster so endpoint identity remains consistent.

Moving routing also ignores persistent deadend masks and does not create new
persistent marks. It retains failed child edges only within the current bounded
search so the search can return to a parent and select another exit. A moved
endpoint changes the routing problem, so only current physical leases, temporary
per-search failures, and loaded crossbar connectivity constrain its retries.
Every routed destination node inside the grounding radius is retained as a
docking candidate before ordinary forward expansion. Thus a legal exit that
leads away cannot hide another route from the same bounded search to a free
destination entry; failed docking returns to the parent decision and tries the
remaining exits.

A failed bounded Generic continuation in Moving never rolls back one committed
hop. Each placement gets a bounded Generic/Fanout routing slice; if incident
routes remain incomplete, the scheduler relocates the focused sink and retries
only that sink's route suffixes.

Moving invalidates a relocated fanout sink by releasing only its private suffix.
The shared trunk and every sibling branch retain their fragments and leases.
Fanout work waits only for a pending Generic seed from the same physical source
pin. When a focus has no unfinished incoming route and only drives unfinished
downstream loads, its placement is finalized and those loads become later move
targets; the completed driver is never moved again.

Focused Moving routing may use the normal safe takeoff or grounding preemption
rules when that is necessary to route the selected cell. Restored sibling work
that is not part of the active focus may not preempt another route tree. This
prevents background repair from repeatedly invalidating unrelated completed
focuses while still allowing the selected relocation to resolve congestion.

For each moved task set, Moving routing repeats the same two routing commands in
order: first Generic routing for the affected source trunks, then Fanout routing
for the affected secondary sinks. A moved instance is not considered finished
until the affected Generic and Fanout tasks are both complete. If these tasks
advance partially, the placement is kept and the next pass continues routing
from that state; the instance is moved again only when the affected task set is
actually blocked.

The focused move is atomic. Moving must not restore deferred tasks or select a
different endpoint while any route incident to the current cell or packing
cluster remains incomplete. If the deterministic placement sequence is
exhausted, it restarts for the same focus; the stage time limit bounds the
overall attempt without exposing partially unrouted incident work.

The complete Moving subsequence is:

1. Select one unfinished sink or strict packing cluster as the focus.
2. Detach each moved sink route at its shared-prefix boundary and release only
   the private suffix leases.
3. If a physical source in the cluster moved, atomically invalidate its whole
   source tree using the Generic-seed/Fanout-sibling rule above.
4. Move the cluster to the next legal placement and rebuild its endpoint pin
   mappings.
5. Run Generic routing for every affected source tree that lacks a trunk.
6. Run Fanout routing for every affected secondary sink.
7. Keep partial committed progress and continue bounded passes at this placement.
8. If incident work remains blocked after the placement slice, release the
   affected private suffixes, move again, and repeat steps 4 through 7.
9. Mark every cluster member finished only after all incident physical bindings
   are complete.

A finished mark is conditional, not permanent. Later source-tree preemption or
shared-prefix repair can invalidate a route incident to that cluster. Before a
finished mark blocks relocation, Moving rechecks all incident bindings. If any
is incomplete, it clears the mark from every cluster member and schedules the
subsequence again. This prevents a stale success marker from causing hundreds
of no-op retries.

## Clock Routing

Clock routing is independent from the three ordinary routing stages. It runs
after Generic, Fanout, and Moving routing and is implemented by
`RouteClocks`. Ordinary route scheduling and its persistent deadend masks are
not used for clocks.

The device database identifies clock-capable site pins, dedicated buffer sites,
programmable crossbar nodes, and cross-tile continuations. Database loading
converts these records into the same numeric node roles and masks used by the
rest of the FPGA model. Runtime clock routing does not inspect tile or wire
names. It traverses `local_src`, `local_joint`, `local_local`, `src_joint`,
`dst_by_src`, `joint_src`, `joint_local`, `joint_joint`, `dst_src`, `dst_local`,
and `dst_joint` according to the current node role.

Some dedicated wires pass through tiles without a programmable switch in that
tile. The loader represents those segments in `local_by_local`. Each entry
stores a numeric coordinate delta, target crossbar type, target node role, and
target-node mask. Construction starts from numeric nodes in explicitly loaded
dedicated-fabric types and expands only their connected tile-connection
components. This permits pass-through interface nodes to be allocated without
importing unrelated database wires. If the landing wire is already a `DST`,
`SRC`, or `JOINT`, its existing role and number are retained instead of creating
a duplicate local node.

For each declared clock, the router places its dedicated buffer in a compatible
site whose input and output pins have numeric endpoint mappings. It then routes
the source-to-buffer connection and builds one shared output tree from the
buffer to every clocked sink. When a source pin has several legal route-tile
locals, the router tests their numeric connectivity and commits the first root
that reaches the initial sink. Later sinks extend the same leased tree, so
shared clock resources are represented once while each sink route retains its
complete source-to-pin path.

Clock routes are stored as typed route edges plus source and destination tile
pins. A route edge records both endpoint coordinates, node roles, and numeric
node values. This keeps design-state serialization architecture-neutral while
preserving enough physical identity for an external architecture exporter.

## Preemption

Preemption is allowed only when a route is blocked by already leased crossbar
nodes and a safe transit victim exists. It must not steal a local start or final
local entry from another route that is using the tile as its endpoint.

### Takeoff Preemption

Takeoff preemption is allowed only for a route start at depth 0 and only after
the start local cannot leave the tile through any currently usable direct or
joint path.

The preempted route must be a transit route through the tile. A local-to-exit
route is not a legal victim for takeoff preemption, because it is also starting
from the same tile-local resource class.

After preemption, the victim net is queued for rerouting and the starting route
may use the freed exit node.

### Grounding Preemption

Grounding preemption is allowed only for the destination nodes that can connect
to the requested sink local. Before preempting, the router checks the complete
numeric `dsts_reaching_local` mask. If any destination node in that mask is
free, no preemption is allowed and routing must use or dock to that free entry.

When every usable destination node is leased, the router may displace exactly a
destination node owned by a transit route. A route that uses its destination
node to connect to a local resource in the same tile is an endpoint route, not
a legal victim. The selected transit tree is unrouted and queued again before
the current route retries grounding. Proximity to the destination or docking
radius alone never permits grounding preemption.

Endpoint-owned destination nodes and joints are never grounding victims, even
when an endpoint has another topologically possible entry. Allowing two local
routes to exchange the same joint creates a deterministic preemption cycle.
After a legal transit preemption, the current route retries grounding immediately
and must claim the released terminal path before the victim task is scheduled.
If a physical node is replicated in several bindings of a shared route tree,
all transit owners of that node are removed atomically before this retry.
Before mutating any victim, the bounded docking search exhausts its free terminal
paths and reports the exact busy destination entry reached by its forward frontier.
Only that physically reachable numeric entry is eligible for preemption; routing
does not speculatively clear unrelated terminal leases or rerun a broad probe.

Grounding may remove either a private Fanout suffix or an entire transit source tree.
When a transit trunk is removed, its Generic seed and dependent Fanout branches are requeued atomically.

### Grounding Docking

Grounding docking is a final fallback after normal final entry and grounding
preemption both fail. It handles the case where the route can reach the
destination area and the destination local is reachable from some incoming
track, but the current forward track cannot directly enter that destination
local.

Docking searches from both ends without committing speculative wire fragments.
One side walks backward from the destination local through possible incoming
destination tracks. The other side walks forward from the current route frontier
through possible outgoing source tracks. If the two searches meet, the collected
fragments are committed to the route and leased normally.

Each docking attempt owns temporary backward-search memory keyed by crossbar
coordinate and destination-node position. Once a backward position has been
expanded without completing docking, another branch does not expand that same
position again. These temporary docking deadends are discarded when the docking
attempt returns and never modify the persistent crossbar deadend masks.

The same temporary state builds one numeric reverse jump index for the bounded
docking window. Incoming transitions are resolved from loaded `dst_by_src`
mapping once and grouped by destination `(x,y,dst)`. Expanding another backward
position performs a lookup in this index instead of rescanning every tile and
source mapping in the window.

The search is intentionally bounded. Both walkers stay within a small square
around the destination tile and the recursion depth is limited. Docking is still
fully abstract: it uses crossbar masks, dynamic lease state, and device jump
resolution only; it does not inspect vendor wire names.

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

## Direct Resource Routes

Some source and sink pins are connected by tile-local or dedicated resource
fabric rather than by crossbar source nodes. If a source pin has mapped output
nodes but no routable crossbar output candidate, the router may represent the
connection as a direct resource route. The route contains tile-pin fragments for
the source marker and final sink pin; only the final sink pin is leased.

This rule is generic: it depends only on abstract source/sink pin mappings and
absence of a routable crossbar output candidate, not on any vendor-specific
resource name.

## Regression Tests

Routing regressions live in `src/tests/fpga` and are registered with CTest.
Run all suites with:

```sh
ctest --test-dir build --output-on-failure
```

Except for the explicit database subtype test, fixtures use synthetic element,
node, and tile names. This keeps the routing behavior under test independent of
any vendor vocabulary.

### `fpga.routing` - `routing_test.cpp`

This is the broad isolated routing-policy suite. It verifies:

- persistent deadends are read and written only by Basic routing;
- a failed child edge returns to its parent in Basic, Fanout, and Moving modes;
- saturated Fanout branch points are skipped and another point is tried;
- a failed current target entry is not immediately retried as if it were new;
- failed near-target docking does not reduce the ordinary continuation depth;
- direct and joint-mediated outgoing source indexes use consistent node bits;
- disconnected double-joint paths are rejected;
- all sixteen abstract FF input positions of a two-slice tile remain distinct;
- connected packed MUX inputs become void tile-internal nets;
- Generic mode emits one route from one physical source port;
- Fanout mode branches from an existing trunk away from the source tile;
- Moving mode releases old cell routes and reroutes the affected hierarchy;
- a route can escape a blocked radius through a one-tile trail initially
  pointing away from its destination;
- bounded continuation commits successful progress and never rolls it back;
- full-net unrouting clears source, transit, joint, local, and tile-pin leases;
- grounding preemption releases the complete victim terminal state;
- remote endpoints require crossbar fabric while attached resource tiles share
  the corresponding route-tile state;
- removing one Fanout suffix preserves its parent destination lease;
- Fanout forks require a destination node already belonging to their trunk;
- equal textual route names retain distinct physical endpoint bindings;
- reattaching one exact endpoint identity does not duplicate or replace a
  completed physical binding;
- moved sinks retain only their reusable source or shared trunk prefix;
- an empty moved-sink binding is still invalidated and scheduled;
- shared terminal Fanouts release only their private local endpoint;
- a failed Generic seed rotates to another sink of the same source port;
- unfocused Moving repairs cannot preempt, while focused Moving and Fanout
  continuation retain safe grounding preemption;
- generated endpoint chains move in dependency order with their real anchor;
- lease release preserves Basic deadends while clearing every owned node class;
- preemption-cycle guards expire at pass boundaries;
- busy transit exits remain visible to preemption candidate selection;
- Basic keeps preempted secondary siblings deferred as Fanout work;
- failed Fanout branches advance their branch-point rotation exactly once.

The suite also runs 64 randomized variants each for direct transit takeoff
preemption, joint-mediated preemption metadata, and preference for a free joint
exit over preempting another route.

### `fpga.grounding_preemption` - `grounding_preemption.cpp`

This suite isolates final-entry ownership and proves that grounding:

- does not preempt while any physically reachable destination entry is free;
- never selects a destination used by another local endpoint;
- does not let an unreachable free destination hide a genuinely blocked entry;
- does not exchange endpoint-owned joints between competing local routes;
- retries and claims the released grounding path before scheduling its victim;
- atomically removes all transit owners of a replicated physical node;
- skips a victim whose removal cannot enable the attempted docking path;
- removes and requeues a transit source tree as one Generic seed plus Fanouts;
- selects exactly the transit-owned destination when all alternatives are
  endpoint-owned.

### `fpga.moving` - `moving_test.cpp`

This suite covers the Moving scheduler and the most recent task-loss fixes:

- moving one Fanout sink releases only its private suffix and preserves sibling
  branches and shared leases;
- destination ownership is charged to the actual landing node;
- each placement receives a bounded routing slice while partial routes advance;
- a focus relocates when one incident route stalls or wanders without completion;
- only Fanouts from the same physical source wait for a pending Generic seed;
- a finished focus remains fixed only while all incident routes are complete;
- route invalidation clears stale finished marks and reopens the whole cluster;
- atomic source-tree invalidation requeues already-empty siblings as well as
  bindings whose leases were released by the current call;
- a restored source tree without an external seed contains exactly one Generic
  task followed by Fanout tasks, including bindings stored in later logical net
  objects; a surviving external seed keeps all restored tasks in Fanout mode.

### `fpga.packing` - `packing_test.cpp`

This suite validates placement legality that routing relies on:

- LUT-to-MUXF7, MUXF7-to-MUXF8, and MUXF8-to-FD chains require real neighbor
  connectivity and one compatible tile;
- unplaced strict-chain sinks reserve future driver and successor lanes;
- multiple MUX drivers share the required lane and all must be packable before
  the sink is accepted;
- future MUXF8 lanes require a compatible sibling MUXF7 and reject occupied,
  disconnected blockers;
- a tile exposes sixteen distinct FD positions;
- recursive blocker discovery sees distant LUT-to-FD conflicts through carry
  and permits connected or independent combinations correctly;
- independent and chained extra LUT inputs have different blocker behavior;
- independent input pins cannot alias one local or mandatory joint;
- unreachable entries do not hide mandatory joint ownership in either packing
  order;
- resource and route tiles share mandatory-joint ownership;
- unrelated locals cannot hide the exact route endpoint needed by a packed pin;
- equal local numbers in different route namespaces remain distinct;
- colliding resource IDs retain distinct pin identities;
- optional two-joint entries avoid joints reserved by another packed input.

### `fpga.passthrough` - `passthrough_test.cpp`

This suite verifies source and target passthrough insertion for LUT, extra LUT,
MUXF7, MUXF8, and FD column positions. It checks that generated elements are
placed in the endpoint tile, that one side of the replacement connection is a
void tile-internal net, and that unrelated LUT overlays are rejected. It also
protects against treating an empty passthrough attribute as generated metadata,
aliasing equal position bits from different element types, or merging distinct
MUX input lanes.

### `fpga.angle_priority` - `angle_priority.cpp`

This suite verifies numeric jump ordering. Loaded deltas override an encoded
shape, the closest destination angle is considered before a wrong direction,
shorter wires win at equal angle, and a longer correctly directed jump wins over
a shorter wrong-angle jump. It covers forward versus opposite directions,
mostly horizontal targets, busy/deadend filtering, and randomized sparse masks.

### `fpga.cb_names` - `cb_names_test.cpp`

This suite randomizes separate local, joint, destination, and source namespaces,
then follows every set bit in `local_src`, `local_joint`, `dst_src`, `dst_local`,
`dst_joint`, and `joint_src` back to its expected node identity. It proves that
endpoint destinations are not mislabeled as jumps and that a multi-tile
passthrough chain has matching step-by-step `dst_src` and `dst_by_src` entries.

### `fpga.dst_by_src` - `dst_by_src_test.cpp`

This suite protects exact one-hop jump resolution. Segment wire names do not
resolve before a switchable endpoint, local endpoints are not transit
destinations, wide loaded deltas do not drop their source mapping, and prefixed
wire families retain their distinct numeric mappings.

### `fpga.subtypes` - `subtype_test.cpp`

This is the real-database reverse test. It loads the configured device database,
reconstructs crossbar subtypes, and verifies every resolved source/destination
entry against the raw tile-connection records that produced it. It also checks
a focused backward index. The suite detects missing, invented, duplicated, or
wrong-coordinate subtype mappings and has a longer CTest timeout because it
loads the full database.

### `fpga.docking` - `docking_test.cpp`

This suite verifies bidirectional grounding docking:

- 20 randomized occupied arenas retain one forced free path that docking finds;
- backward search can meet an existing forward anchor destination;
- docking ignores persistent Basic deadends;
- docking steps out of a destination tile whose current arrival cannot reach
  the required local;
- edge endpoints use the larger configured docking window;
- backward expansion uses the resolved destination namespace;
- only a physically reachable blocked terminal is reported for preemption;
- 20 randomized cases build valid multi-hop backward routes;
- failed backward positions are memoized within one attempt and the reverse
  mapping window is indexed only once.

### `fpga.backwards_resolve` - `backwards_resolve.cpp`

This suite builds 20 randomized 14-by-14 devices. Each contains hundreds of
numeric source-to-destination mappings, multi-destination sources, invalid
destination bits, and wrong target subtype IDs. It compares the generated
reverse index and mapping scan count against a reference implementation, both
with and without a source-coordinate filter.

### `fpga.pnr_db` - `pnr_db_test.cpp`

This suite round-trips placed instances and single-sink, forked, and deeper
route-tree hierarchies through the design database. It compares source/sink
identity, node IDs, coordinates, kinds, names, edges, branches, and arbitrary
wire annotations, checks graph ID consistency, and rejects third-party hardware
names from the synthetic fixture.

### `fpga.arena` - `arena_test.cpp`

This suite runs the routing entry point on a synthetic 20-by-20 fabric with
border endpoints, randomized occupancy, and 50 Generic routes. It requires all
routes to finish through bounded incremental passes. A focused Moving case also
proves that a blocked docking entry returns to its parent and selects another
free destination instead of retaining the blocked prefix.

### `fpga.repair_prefixes` - `repair_prefixes.cpp`

This suite creates valid shared route trees, randomly damages ownership and
prefix state, and verifies that repair returns each damaged source hierarchy to
one Generic seed plus all dependent Fanouts without touching an unrelated
control tree. It checks that old route storage and leases are released. It also
proves that repeated entries into one logical stage share one cumulative timeout
budget and that early scheduler exits still charge elapsed time.

### `fpga.clock_routing` - `clock_routing_test.cpp`

This regression constructs a vendor-neutral dedicated component with a
pass-through tile and verifies that numeric local transitions cross it. It also
checks that a dedicated local landing on an existing destination node retains
the destination role, and that an unrelated tile-connection component is not
imported into the dedicated routing graph.
