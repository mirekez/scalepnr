# Routing Rules

This document records generic routing behavior that must stay independent of
any FPGA vendor database.

## Main Routing Principle

ScalePNR uses two distinct approaches to tracking a route through the tile
crossbar mesh:

1. **Direction-driven routing.** Every available jump out of a crossbar is
   visited in geometric angle order around the requested direction: west,
   northwest, north, northeast, east, southeast, south, and southwest. Shorter
   jumps have priority within the same angle. This makes another visit to the
   same tile unlikely during ordinary operation, while still permitting it
   when congestion forces the route to wander. Only direction-driven engines
   do not check how many times the current candidate has already passed a
   crossbar tile.
2. **Combinatorial routing.** The engine recursively or iteratively considers
   all usable alternatives, subject to the search's documented spatial, depth,
   time, or width limits. Every candidate path carries its own tile-visit
   counts. It may pass the same crossbar tile once or twice, but its third visit
   to that tile is rejected. Counts are path-local: rejecting one branch must
   not suppress an independent branch that has not used the tile twice.

Every routing stage and fallback must identify which approach it uses. Basic
and Fanout routing are direction-driven until their Grounding/Docking fallback,
which is combinatorial. Moving sources, destination-to-source routing,
backward parking against retained anchors, and Moving destinations are
combinatorial. Clock routing is combinatorial tree search. Const routing is a
combinatorial local-graph search confined to one crossbar, so it passes that
crossbar only once by construction.

## Top-Level Routing Requirements

The following rules are the primary routing contract. They are requirements,
not implementation suggestions, and the detailed rules later in this document
must preserve them. Routing has exactly six ordered stages: Clock, Const,
Basic, Moving sources, Fanouts, and Moving destinations. Each stage owns
multiple bounded passes; a pass is work inside a stage and is not a stage
itself. Clock and Const are mandatory exceptional stages. They complete their
own route classes before ordinary routing starts and never classify their nets
as trunks or fanouts. The Basic stage is called Generic routing in the current
implementation and in the detailed sections below.

### 1. Clock Routing

Clock routing is a combinatorial tree search and runs first. It routes every
declared clock through
database-identified dedicated resources and must finish with zero unresolved
clock sinks. Completed clock trees are protected leases visible to every later
stage. Clock tasks never enter the ordinary trunk, fanout, or movement queues.

### 2. Const Routing

Const routing is a combinatorial tile-local search and runs second. It routes
every load of VCC, GND, and equivalent
database-declared constant sources from the independent constant root in the
load's route crossbar. Every sink is a separate physical route root even when
the logical signal is shared. Const routing must finish with zero unresolved
loads. Its protected routes never enter the ordinary trunk, fanout, or movement
queues.

### 3. Basic Routing

Basic routing is direction-driven until its combinatorial Docking fallback. It
performs an initial, brief allocation of routing resources for
every net. For a multi-fanout net, it must select and route only one initial
trunk, as far as the bounded Basic passes permit; the remaining sink branches
are deferred to Fanout routing. Basic may finish with an incomplete trunk, but
it may not finish with a net that has no Takeoff.

Only Basic routing may learn and enforce persistent `SRC` deadend marks. A
`SRC` is marked as a deadend when no continuation after that node is free
because every possible continuation is occupied. These marks expose congested
areas and must prevent later Basic searches from entering those areas.

Grounding is the final search step for a Basic route. It uses the special
Docking algorithm, ignores persistent deadend marks, and performs a full
combinational search in both directions within a 5-by-5-tile window. One search
frontier grows forward from the routed prefix and the other grows backward from
the destination.

Within a tile, Grounding (`DST` to local) and Takeoff (local to `SRC`) may
preempt a transit route that occupies a required resource. A preempted victim
must be unrouted and queued for routing again. The safe default is to unroute
the complete victim route; an implementation may preserve a prefix or other
part only when that part is ownership-separable and retaining it cannot leave
stale leases or an invalid route tree.

Basic may hand unresolved trunk tasks only to Moving sources. Their incomplete
prefixes remain leased and become exact anchors for destination-to-source
docking. Completed trunks stay leased, and Basic never releases deferred
suffixes directly to Fanouts.

### 4. Moving Sources

Moving sources is combinatorial, including destination-to-source search and
backward parking against retained route anchors. It receives only trunks that
Basic could not complete. It ignores
persistent deadend masks and routes backward from the fixed destination before
changing placement. The reverse search follows the numeric incoming-jump index
until it reaches a nearby route tile where a legal source placement can drive
the selected `SRC`. Only then does it invalidate the old source tree, move the
physical source cell, and commit the already-proven trunk. Other bindings from
that source remain parked suffixes.

An existing partial forward route is the first recovery target. Reverse routing
tries its newest landing first and docks to the latest reachable numeric anchor,
preserving the source takeoff and useful prefix. If no anchor is reachable, a
failed prefix is released before replacement search so it cannot remain as
permanent congestion. This release is per binding and occurs only after every
retained anchor has failed; unrelated prefixes remain leased. A speculative
state view hides only the selected route's own leases while its replacement
trunk and source placement are tested.

The reverse walk records every reached frontier in integer distance buckets and
completes the reachable spatial search before probing placement. The stage
deadline remains its execution bound. Probing then proceeds nearest to the old
source first without imposing an old-placement radius: congestion may stop the
reverse path before it enters such a radius. A rotating window checks at most
eight route-proven placement candidates per retry. This requires no runtime
sorting. If the existing placement can drive the reached exact takeoff, the
trunk is committed without moving or disturbing its inputs. Otherwise the
proven route is retained and the source moves to the
nearest reached location whose packed resource and takeoffs are legal. There is
no arbitrary source-distance cutoff that can discard the only proven route.
Packing probes use a bounded window with a persistent per-source cursor, so a
retry continues after the previously rejected frontiers instead of rescanning
them or evaluating every reached placement in one scheduler turn. When both the
numeric reverse frontier and all of its placement windows are exhausted, the
search reports that exhaustion directly. A successful route clears the probe
cursor.

When a trunk starts at a generated tile-local passthrough, the route endpoint
remains the passthrough but relocation follows its void source chain to the real
physical driver. Generated adapters are never selected as independently movable
source cells. The complete strict packing cluster is previewed with the external
route endpoint fixed at each candidate element lane. A candidate is accepted
only when the preview assigns every physical and generated cluster member.

The stage starts with a fast anchor-only sweep over every unfinished trunk and
repeats that sweep after each Generic recovery chunk. Each reverse search
checks retained prefix landings
through a per-tile numeric destination index. Successful suffixes are committed
immediately. Failed prefixes are collected and released in batches of 1024,
after every retained anchor for that trunk has failed. Independent unresolved
drivers are then moved in bounded route-first batches.
Every successful backward probe commits its trunk and reroutes the moved
cell's inputs synchronously, so the stage continues directly with another
route-first batch. Generic recovery is triggered after 256 prefix
releases, 32 physical source moves, or 1024 changed reroute tasks. Pending
preempted tasks also trigger recovery so small queues cannot starve below these
thresholds. It processes
at most 4096 rotating tasks before returning to route-first work.
When relocation completes the last trunk, it must still execute the stage
handoff and release deferred suffixes to Fanouts. Empty active work alone is
not routing completion; pending and later-stage queues must also be empty.
Prefixes created and rejected by that recovery chunk are cleanup from the same
attempt and do not immediately trigger an identical second chunk. The initial
anchor sweep's releases do count because Basic has not retried that newly freed
capacity; later recovery is driven by accumulated source moves or other route
topology changes.
If a locally valid placement fails while its input routes are reconnected, the
transaction restores the old placement, routes, leases, and queues exactly and
records that placement as tried. The fair source queue tests another
route-guided placement in a later recovery cycle instead of repeating the
reverse search inside one task.
Each batch bounds both accepted relocations and rejected source probes to one
batch width, so a sparse late-stage queue cannot consume the stage deadline
while trying to fill a success quota.
If a complete batch is rejected, its trunks rotate behind every untouched
trunk. Rejection alone does not trigger a Generic queue scan; accumulated
topology changes do. Moving Sources never applies the one-focus-at-a-time
deferred activation used by Moving Destinations.
These trunk retries retain Generic takeoff, bridge, and grounding preemption;
only persistent Basic deadend masks are ignored. Unfocused destination repair
does not receive this broad preemption permission.
`routeOutTry()` validates the hypothetical placement without moving the live
route endpoint. Every connected output port on that endpoint must receive a
distinct free resolved direct or joint-assisted takeoff, and the trunk port must
drive the exact `SRC` reached by the backward route. Candidate resource tiles
come directly from the selected route tile's attached-resource index; no radial
grid scan or vendor-name lookup occurs during routing. A structurally valid
output local alone is insufficient. Before a cell is moved, every ordinary
input is routed numerically against a private copy of the affected crossbar
states. The complete proposed output trunk is reserved in that private state
**before** searching any input; each successful input is then reserved before
searching the next. Thus the searches themselves avoid competing for the same
resources instead of detecting the collision only afterward. Failed input
probes are remembered per placement and output takeoff, not per placement alone,
so a different output route can still use that placement. The output path is
materialized only after cheap packing/takeoff checks and is reused for commit.
These proofs create no live leases;
only the selected proof paths are retained for commit after relocation.
Candidates at the current unchanged placement skip this input proof because no
input endpoint or route is invalidated. After placement, all retained input
proofs are committed immediately; the source stage does not defer this step
while unrelated tasks consume the validated capacity.
These focused validation reroutes may preempt an unrelated transit suffix when
the normal takeoff or grounding rules prove it is the only usable corridor.
Preempted foreign work survives a rejected-placement rollback and remains in
the scheduler; only work created by the rejected cluster transaction is erased.
If an input remains incomplete, candidate rejection is atomic: the prepared
trunk and candidate-local input suffixes are removed, every packed cluster
member returns to its exact old tile position, and the prior binding storage,
route vectors, tile masks, ownership registrations, and scheduler queue sizes
are restored. A rejected placement must not manufacture new trunk tasks.

Moving a source can invalidate routes entering the moved cell. Any resulting
missing trunks are retained within this stage, while suffixes and endpoint-local
distributed-source work remain parked for their later stages. Moving sources is
a mandatory barrier: it must finish with zero active trunks, zero deferred
trunks, and no active relocation focus. A timeout or nonzero exit is fatal.
Fanouts cannot start until every parked suffix has a completed physical source
exit.

### 5. Fanout Routing

Fanout routing is direction-driven until its combinatorial Grounding/Docking
fallback. It starts only after Moving sources reaches zero and routes every
deferred suffix of each multi-fanout net from the trunk state established by
Basic or repaired by Moving sources. Fanout routing ignores persistent deadend
masks. It may still preempt a transit route when that route blocks Grounding,
subject to the same victim-safety and requeue requirements.

Successive fanout suffixes must grow from successive downstream tiles of the
trunk, beginning with the next available trunk tile. After all trunk tiles have
been used as suffix sources, Fanout routing must reuse earlier trunk tiles as
branch sources for the remaining suffixes. Failure to complete a suffix leaves
that suffix explicitly scheduled for Moving; it must not discard the suffix or
create another source-tile Takeoff. A trunk invalidated by Fanout preemption is
repaired as Generic work inside the Fanouts stage without re-entering Basic or
consulting Basic deadends.

### 6. Moving Destinations

Moving destinations, also called Fanouts moving, is combinatorial. It is
responsible for completing every suffix that remains
unfinished after Fanout routing. It selects an unrouted, congestion-blocked
destination cell and uses a fast radial search to move that cell to a nearby,
less congested legal location. It ignores persistent deadend masks.

Moving a destination must unroute only the suffix required by that destination.
It must preserve the shared trunk, sibling fanouts, and every other safely
separable routed prefix. After the move, each affected unrouted suffix is routed
to completion with the Generic and Fanout algorithms in sequence. Moving
destinations continues in multiple passes until no unrouted suffix remains.

## Constant Routing

Logical constant inputs remain attached to the design's global constant
connections. A crossbar database may declare continuously driven local nodes in
`constant_one_nodes` and `constant_zero_nodes`. `RouteVCC` discovers those
logical loads, creates stable logical source/net identities, and prepares any
required tile-local passthrough endpoints. It does not search paths or lease
nodes.

`RouteDesign` routes every prepared load during the dedicated Const stage. A
constant task is neither a Generic trunk nor a Fanout: every target crossbar
provides an independent physical root. The stage uses the numeric local graph,
commits each root-to-sink path, and must reach zero before Basic begins. A
blocked constant is a fatal Const-stage routing failure; it is not deferred to
either movement stage. The failure path writes a visualization centered on the
blocked sink and highlights the failed logical route.
The local path search uses only the numeric `local_local`, `local_joint`,
`joint_joint`, and `joint_local` masks. Runtime routing does not inspect node
names or architecture-specific text.

When a generated constant endpoint feeds a packed multi-column element chain,
placement reserves the predecessor lane selected by the target port. Rehoming
accepts a shared output-local identity only when the loaded element graph proves
transitive strict-chain connectivity between its owners.

The physical net is protected from preemption by unrelated nets because its
local root is a mandatory continuously driven resource. A final audit uses the
original prepared task identities, not only surviving route bindings, so a
removed branch cannot silently disappear after later routing stages.

One logical distributed source may produce several disconnected physical
roots. The design-state database writes each connected root as a separate route
tree while preserving the common logical net and source identity. This keeps
route trees structurally connected and lets an architecture-specific exporter
map the generic constant source metadata to its physical static-net source.

## Route Stages

Routing is split into the six top-level stages defined above. A stage is the
scheduler phase; passes are the lower-level bounded iterations run inside the
currently active stage. The order is Clock, Const, Basic, Moving sources,
Fanouts, then Moving destinations. Each stage owns an absolute wall-clock
deadline from its first entry; focused relocation and queue-maintenance work are
therefore included in the same budget as path search. Clock and Const may finish
in one pass when their dedicated algorithms complete the full workset directly.

The default hard budget is 20 minutes per stage. Independently, a progress
watchdog samples committed outstanding work in one-minute windows. A window
must retire at least `ceil(1% * tasks_at_window_start)` tasks; three consecutive
deficient windows cancel the current search. In Fanouts, pass finalization
conserves all unfinished work and hands it to Moving destinations immediately,
even before the pass-count handoff threshold. Other stages terminate the run as
failed routing. A qualifying window
resets the deficient-window streak. Search and relocation cancellation points
poll the same watchdog, so one long speculative operation cannot hide a stalled
stage until its hard deadline. The window and streak are configurable through
`SCALEPNR_ROUTE_PROGRESS_WINDOW` and `SCALEPNR_ROUTE_STAGNANT_WINDOWS`.
On failure, scalepnr overwrites `routing_failure.png` and
`routing_failure.txt` in `SCALEPNR_FAILURE_ARTIFACT_DIR` (or the current
directory when unset). The reported task is selected from live unfinished
queues, excluding completed bindings and tasks marked for retirement. Retained
failed-search paths may illustrate that task but cannot select a different,
already-completed net. Thus the image and textual diagnosis stay beside the run
that produced them without accumulating stale reports. A new routing run
removes this pair before starting, including when that new run succeeds.

Focused movement resolves incident nets from each endpoint's numeric connection
designators through a per-module index. Candidate checks, invalidation, anchor
collection, and completion audits iterate that incident set instead of scanning
every route binding in the design. The index is generic design state and does
not depend on node names or architecture-specific text.

Every physical route is identified by its logical net, driver instance and pin,
sink instance and pin, and route name. A route name by itself is not an identity.
Attaching the same exact endpoint identity updates its incomplete route-vector
owner instead of creating another task. A completed physical binding is never
replaced by a later incomplete attachment.

A route is complete only when its committed fragments reach the required sink
tile pin. Partial crossbar progress is retained as a prefix, but it is not
reported as a completed route binding.

### 3. Generic Routing

Generic routing builds the first route for each driver output port. Task
collection walks sink input ports and follows each connection back to its
driver. It marks each source port after emitting its first task and defers later
fanouts from that same source port. This prevents many sinks of one source from
all trying to start at the same source tile and consuming unrelated exits before
a trunk exists.

Source-tree identity is the canonical physical driver instance and output pin,
not a logical net string. Generated source-passthrough chains are followed to
that upstream endpoint before indexing. Consequently, split logical aliases of
one physical signal receive one Generic trunk, are never considered foreign
preemption owners, and are invalidated and requeued atomically as one tree.

Before search, packed source endpoints are normalized to the fabric-facing end
of their generated element chain. Every output connection caches its next
physical endpoint, and insertion transfers the complete logical fanout in one
linear operation. Topology preparation does not unroute the net itself. The
scheduler then releases the indexed physical source tree once, retargets all
bindings and deferred tasks together, and preserves exactly one Generic seed.
This separation avoids quadratic sibling scans on high-fanout source trees.

The search uses only numeric crossbar masks. At an incoming destination or local
source, it enumerates direct and joint-mediated outgoing source bits. Source
bits are visited by angle relative to the destination; for equal angles, shorter
jumps have priority. Once a source bit is selected, `dst_by_src` resolves its
exact numeric landing node and coordinate delta. Textual wire names are never
used to choose a route at runtime.

The first Generic pass reserves one physical takeoff hop for every source-port
seed. This prevents an early long trunk from consuming all exits around source
tiles scheduled later. Every subsequent continuation explores at most the
configured short recursion depth, normally five hops. Speculative suffix
fragments and their leases remain local to that search. A successful suffix is
appended to the route vector and leased atomically. A failed suffix returns to
its parent so another outgoing source can be tried. Successful partial progress
is committed even when the destination is not yet reached, allowing the next
pass to continue at the committed endpoint.

If a pass grows the unfinished Generic queue, takeoff preemption displaced more
existing source trees than the pass completed. Basic hands the conserved state
to later stages immediately, before repeated preemption erases additional
seeds. The configured Basic timeout remains an upper bound rather than a reason
to continue destructive congestion churn.

A blocked committed endpoint removes one incoming committed hop and retries
from its parent. Basic routing deliberately makes no distinction between a
missing physical continuation and one whose resources are all occupied: both
mean that the current frontier has no free exit. Later Basic passes may consume
up to the normal per-task budget of consecutive dead parent hops in one
scheduler turn. Any successful forward suffix ends the task's work for that
pass, so this recovery budget cannot become an unbounded forward search.

After the takeoff sweep, Generic passes visit committed multi-hop prefixes
before one-hop takeoffs and empty routes. The ordering uses stable linear
buckets, not runtime path sorting. It lets useful routed work approach its sink
before displaced tasks consume transit capacity again, while the first pass
still guarantees one takeoff attempt for every source-port seed.

Generic routing runs bounded passes while its unfinished queue is converging.
If a full attempt cannot retain a useful prefix, the task remains scheduled for
another pass. The scheduler must not lose an empty route merely because no lease
was released during its latest invalidation. A logical stage has one cumulative
time budget across all re-entries used to repair invalidated Fanout trunks.

Generic work is conserved, not aborted, when that budget expires, when two
consecutive passes grow the unfinished queue through preemption churn, or when
a pass proves that no task has an active continuation. Unfinished trunks are
handed to Moving sources. Every suffix remains parked until the source-moving
barrier validates all trunks. This handoff never marks an incomplete binding
complete and never drops its endpoint identity. Large state snapshots are
optional diagnostics: stdout-only runs suppress timeout and intermediate
blocked-state files without changing scheduler behavior.

Source deadend masks are learned and enforced only during Basic routing. They
are tile-and-source collision marks: a source is marked after its child search
returns without a committed suffix, whether the failure came from topology or
occupancy. These marks are sticky for the complete Basic stage. Unrouting and
preemption do not clear them. Docking, Moving sources, Fanout routing, and
Moving destinations ignore the persistent Basic masks because they solve
different search problems after occupancy or placement may have changed.

Generic routing needs one completed trunk per physical source port. If the
selected seed sink repeatedly fails after its partial source tree is released,
the scheduler returns that sink to Fanout work and tries another deferred sink
from the same source. One difficult endpoint therefore cannot permanently pin
the source's Generic trunk selection.

### 4. Moving Sources

Moving sources is a trunk-only recovery stage. It consumes the unfinished
Generic queue from Basic and chooses each physical driver as the relocation
target. For each trunk it first grows a free route backward from the destination
pin through the prebuilt numeric reverse index. A bounded placement search is
run only around reached reverse-frontier tiles. Every reached frontier is kept
in a distance bucket and examined nearest-first without sorting; rejecting one
full tile therefore does not discard equal-distance alternatives. Persistent
Basic deadends are disabled before the first source move and remain disabled
for this stage.

For each source, retained Basic prefixes are tested as destination-to-anchor
routes inside that source's complete route-first attempt. The anchor lookup
intersects a per-tile anchor destination mask with `dsts_reaching_src`, then
looks up only the exact `(tile, DST)` landing. A prefix that misses every anchor
is released before the same source performs replacement search. There is no
queue-wide preliminary anchor sweep: it could consume the stage timeout before
any source received a definitive repair result. Partial work created by a later
Moving Sources Generic recovery remains committed and becomes that source's
next route-first anchor. Selected relocation candidates inspect their own
prefix through a temporary lease-free state view, so reverse probing does not
require queue-wide teardown. Complete trunks, dedicated trees, and deferred
fanout suffixes are not changed.

Generic recovery is change-driven rather than batch-driven. A pass is due
after 256 failed-prefix releases, 32 committed source moves, or 1024 changed
reroute tasks, or when pending preemption victims need to join the workset.
Each recovery pass handles at most 4096 tasks and rotates the
remaining workset. Its newly formed prefixes receive another anchor-only sweep,
then at least one route-first batch runs before another recovery chunk can be
scheduled. A rejected relocation batch therefore advances to other source
probes without repeatedly rescanning the full trunk queue.

The reverse route is speculative and owns no live masks while placement is
being selected. `routeOutTry()` uses a non-destructive element-packing preview
and temporary crossbar-state copies to prove simultaneous takeoff for all
connected outputs. The selected trunk output must reach the exact reverse-route
`SRC`; other outputs must each retain a distinct free resolved takeoff.

After this proof, relocating the driver atomically releases its old physical
source trees, moves the cell to the selected element position, and leases the
prepared trunk in forward order. One binding owns that replacement Generic
trunk and additional bindings from the same source pin remain parked as Fanout
suffixes. Routes entering the moved cell are invalidated and rerouted
immediately, before the relocation transaction returns to the outer scheduler.
These validation reroutes use focused preemption. They may displace a transit
suffix, but never an endpoint owner or a route from the same physical source
tree. An input failure removes the replacement trunk and candidate-local input
suffixes and restores the complete packed cluster and its exact prior route
state. Work created by the rejected cluster is discarded, while every foreign
tree displaced by preemption remains queued for repair.
Endpoint-local distributed-source tasks are not movable driver trunks and
remain parked for Moving destinations.

The stage succeeds only when its active trunk queue, deferred trunk queue, and
relocation focus are all empty. A complete route-first source attempt that
cannot find and validate a replacement route is immediately fatal: the router
passes that exact task to the standard routing-failure path, renders its
retained numeric path in red, writes the PNG, and exits nonzero. It is not put
on cooldown or rotated behind another source. The scheduler then validates that
every parked suffix has a completed source exit. A timeout or a suffix without
a source trunk is also an invariant failure; the scheduler does not return to
Basic.

### 5. Fanout Routing

Fanout routing runs after Basic and Moving sources have built every initial
source trunk. It advances through bounded passes inside the Fanouts stage.
Every Fanout task must have at least one already routed source exit. A source
marker or tile-local endpoint is not enough; Fanout mode never routes from the
source tile.

Fanout branch discovery inspects the Generic trunk first. A preferred trunk
fork has more than two free exits, but a lower-capacity usable trunk fork is
still tried before any sibling tree. It materializes a shared prefix only after
a branch succeeds. A failed attempt broadens to one additional routed sibling
tree per retry, avoiding quadratic scans of large fanout hierarchies. Fanout
never re-enters Basic. If Fanout preemption demotes a Generic route, that trunk
is repaired as Generic work inside the Fanouts stage with persistent deadends
disabled; dependent suffixes remain parked until that repair finishes.
Exhausting a partial branch retry window advances both its branch-point offset
and its source-tree retry, so the next attempt cannot silently rebuild the same
failed trunk branch instead of inspecting a completed sibling.

A continuation whose committed private endpoint has no usable exit does not
consume that retry window repeatedly. If the branch has a private parent, only
its last private hop is released and the next pass retries from that parent;
the shared source tree remains leased. If the blocked hop is the branch's only
private hop, the branch is discarded immediately and selection advances to the
next branch point or routed sibling. Failures that are not blocked at their
root retain the normal bounded retry window.

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

Discarding or rotating a fanout must first transfer ownership of any prefix
still used by surviving branches, including branches on another logical net
with the same physical source. Only exclusive resources may be released; Tile
registrations must also be removed for the discarded shared prefix. A shared
fragment's `owns_landing` flag owns only its exact destination Tile and DST
node, never its source-side DST (even when the numeric node IDs match).
The routing regression covers repeated parent/child removal with randomized,
abstract node names and checks all DST, SRC, JOINT, and binding-index leases.

If the trunk has no preferred branch point, the follower records a fallback
point at the end of the trunk. Before using that fallback, Fanout routing checks
already routed sibling routes from the same physical source pin and applies the
same branch search to them. Only after the trunk and routed siblings have no
preferred branch point may fallback points be tried. If no trunk or sibling can
provide any branch point, Fanout routing leaves the task unfinished for Moving
destinations instead of starting a new source-tile route.

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

Fanout docking first tries partial private bridge victims, then may exchange
one completed foreign transit route. Current-source-tree and endpoint owners,
shared bridge ownership, and reciprocal preemption cycles remain protected.
Only the exact conflicting suffix is cut; its valid prefix stays leased and
the displaced binding is requeued for recovery. A completed foreign transit
must not be rejected solely because the current stage is Fanouts.

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
Fanout or Moving-destinations regressions.

### 6. Moving Destinations

Moving destinations runs when Fanout routing cannot reduce the unfinished task
count. It is still a stage, and its relocation attempts are followed by bounded
Generic and Fanout passes for only the affected task set. The router selects an
unfinished sink endpoint or strict packing cluster and tries nearby legal tile
positions using the generic placement legality checks. Ordinary load cells are
the relocation targets; a completed driver is not moved merely because one of
its fanouts is blocked.

At the Moving-destinations handoff, one linear audit removes task records whose physical
bindings were completed by earlier preemption or sibling work and merges task
duplicates. This keeps the persistent queue proportional to incomplete routes;
later focus changes remain incremental and do not repeat the global audit.

If a fanout sink is moved, only its private branch suffix is unrouted. The
shared trunk and sibling fanouts retain their routes and leases. Moving keeps
per-instance tried placement history to avoid cycling through the same failed
positions.

All input suffixes incident to the moved sink are invalidated because every old
sink local belongs to the previous placement. Output source trees are invalidated
only when the moved packing cluster actually contains their physical driver.
Generated passthrough elements connected by void resource nets move with their
owning cluster so endpoint identity remains consistent.

Invalidating one incident source tree can also return sibling branches whose
sinks are outside the moved cluster. The relocation boundary partitions this
returned work immediately: incident routes stay in the atomic focused queue,
while displaced siblings enter the persistent Moving queue once. A sibling may
not remain in the focused queue and trigger a false non-incident handoff before
the moved cell's own routes finish.

When a focus starts, Moving compacts the persistent queue in place and removes
all stale tasks incident to that focus before activating reconstructed tasks.
Unrelated deferred work keeps its order. This prevents each failed placement
from adding another generation of the same incident routes while avoiding a
global route-list rebuild.

One focus receives a bounded slice of eight distinct placements. If incident
routes remain incomplete, Moving returns the complete focused task set to the
persistent queue, keeps its tried-placement history, and gives another endpoint
a recovery slice. This preserves focused routing atomically while it is active
without allowing one congested sink to consume the complete Moving-stage time
budget. A later visit resumes with different placement candidates.

Route bindings reconstruct endpoint identity after every relocation, but they
do not reset the active scheduler state. Moving merges the live route-attempt,
Fanout-branch retry, and branch-offset cursors into each reconstructed incident
task. A blocked Fanout therefore advances to another trunk fork or routed
sibling after relocation instead of retrying branch offset zero indefinitely.
The per-task no-progress counter is placement-local and is reset by relocation;
carrying it to a new tile would reject that tile after its first failed suffix.

One inactive focused pass does not immediately relocate the cell. The same
placement receives the bounded retry window used by its incident tasks, so a
failed suffix can advance to another exit without discarding sibling routes
that already grounded there. Moving relocates only when that task/placement
window is exhausted.

Completing an ordinary incident route renews one bounded placement window and
the per-task windows of the remaining siblings. Every focused pass already
attempts every remaining sibling, so the window is not multiplied by the number
of routes. A distributed constant source does not renew the window because it
can reach the sink independently of the chosen placement. Partial-prefix growth
also does not renew either window, so wandering routes remain bounded.

If a moved sink exhausts every branch point exposed by its completed source
tree, another sink relocation cannot change those branch points. Moving then
atomically invalidates that physical source tree, keeps the driver placement,
and promotes the blocked moved sink to the replacement Generic seed. Every
other binding from the same source port is requeued as Fanout work behind the
new trunk. This recovery is allowed only for an active moved focus and once per
source tree at one placement. The attempt marker is propagated to every sibling,
so another failed sibling cannot immediately invalidate the replacement seed;
the focus relocates instead. This recovery is triggered only after a Fanout
attempt advances its branch cursor without completing or extending the route;
ordinary partial progress never rebuilds the tree.

Focused preemption can create more sibling repair tasks after relocation has
started. The inactive-pass boundary applies the same partition: only incident
routes remain in the atomic focus, while external siblings are conserved for
later Moving focuses.

If every deferred endpoint is temporarily on placement cooldown, Moving
advances the relocation epoch and scans again. It does not run empty routing
passes; a nonempty pool with no cooldown and no movable endpoint is reported as
an invariant failure.

When a completed or yielded focus leaves the active queue empty while deferred
endpoints remain, the next scheduler iteration enters relocation immediately.
Deferred work is never charged as repeated zero-task routing passes.

Before accepting a candidate placement, Moving reserves temporary terminal
paths for every affected input. These reservations are ordered by physical
flexibility: an input whose alternatives all share one joint is checked before
an input that can use several distinct joints. The masks are temporary and
architecture-neutral; this prevents a flexible input from consuming the only
entry resource available to another member of the packed cluster.

The unfinished route that triggers relocation is the primary placement anchor.
When it has a committed partial prefix, the prefix endpoint is used; otherwise
the external endpoint across the moved-cluster boundary is used. The candidate
walk starts there and accepts its first legal placement. Other incident inputs
and outputs remain mandatory terminal-support checks, but multiple output
fanouts cannot average the search origin away from the blocked input that caused
the relocation.

When the moved sink has several distinct incoming routes, their route anchors
bound one shared search region and Moving starts at the center of that box.
This prevents relocation from alternating between individually convenient
input locations while invalidating a route completed at the previous location.
Output fanout endpoints do not participate in this balance, so they still
cannot pull a high-fanout driver away from its blocked incoming route.

Moving destinations also ignores persistent deadend masks and does not create new
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

Focused Moving-destinations routing may use the normal safe takeoff or grounding preemption
rules when that is necessary to route the selected cell. Restored sibling work
that is not part of the active focus may not preempt another route tree. This
prevents background repair from repeatedly invalidating unrelated completed
focuses while still allowing the selected relocation to resolve congestion.

For each moved task set, Moving destinations repeats the same two routing commands in
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
8. If incident work remains blocked after the bounded passes for one placement,
   release the affected private suffixes, move the same focus again, and repeat
   steps 4 through 7. Do not expose its incomplete incident work to another
   focus between relocations.
9. Mark every cluster member finished only after all incident physical bindings
   are complete.

A finished mark is conditional, not permanent. Later source-tree preemption or
shared-prefix repair can invalidate a route incident to that cluster. Before a
finished mark blocks relocation, Moving rechecks all incident bindings. If any
is incomplete, it clears the mark from every cluster member and schedules the
subsequence again. This prevents a stale success marker from causing hundreds
of no-op retries.

## Clock Routing

Clock routing is the first top-level stage and is implemented by `RouteClocks`.
It is independent from the four ordinary routing stages and from Const routing.
Ordinary route scheduling and its persistent deadend masks are not used for
clocks.

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

Later placement recovery never converts a clock branch into an ordinary route
task and never releases it for use by Basic or Fanout routing. If relocation
changes a clock sink, the old protected tree remains reserved until all
ordinary placement is stable. The clock router then replaces the affected
clock trees in one dedicated repair transaction and verifies zero unresolved
sinks. This repair belongs to Clock routing; it is not a seventh stage.

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

Grounding preemption is allowed only for complete terminal paths that connect a
physically incoming destination node to the requested sink local. Before
preempting, the router checks every numeric `DST -> [JOINT...] -> LOCAL` path.
If one path has a free destination and every required joint is also free, no
preemption is allowed. A free destination whose required joint is occupied is
not incorrectly treated as a usable terminal path.

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
Only that physically reachable numeric path is eligible for preemption. Every
leased destination and joint on the path must be owned exclusively by transit
routes; otherwise the path is protected. Routing does not speculatively clear
unrelated terminal leases or rerun a broad probe.

The forward and backward docking frontiers remain separate until they meet on
the same numeric destination position. If one occupied transit edge is the only
connection between the two reached sets, docking reports that concrete
`dst -> joint(s) -> src -> dst` bridge. The router verifies every binding using
each busy bridge node before changing state. A bridge is preemptible only when
all of those bindings use it as transit; a source endpoint, destination endpoint,
protected route, or unidentified lease rejects the candidate.

An occupied bridge may begin directly at the committed forward anchor. Docking
therefore resolves occupied source bits through numeric `dst_by_src` just like
free candidates, but does not traverse them. It reports such an edge only when
its resolved landing is present in the independently reached backward frontier.
This permits exact transit preemption without treating every busy anchor exit
as a useful bridge.

Bridge preemption detaches each victim at the earliest busy node in that exact
bridge. This preserves its routed prefix and shared source tree while releasing
only the suffix that prevents the frontiers from joining. All bindings that
replicate the same physical bridge are cut together, requeued, and the current
docking attempt immediately retries against the released edge.

During Fanout or Moving-destinations routing, the bridge victim must already be partial. A
completed route is immutable in these stages because exchanging one completed
route for another does not reduce unfinished work and may remove the seed used
by sibling branches. A blocked Moving focus relocates instead. Generic routing
alone may consider a one-for-one completed victim after partial bridge
candidates are exhausted.

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

The backward search uses a bounded breadth-layer beam. Capacity is reserved for
each unexpanded parent in the current layer, not divided permanently among
terminal entries. A terminal entry that has no incoming transition therefore
releases its reservation, while the viable entry can inspect later predecessor
nodes up to the common beam limit. This prevents disconnected terminal entries
from starving a physically routable entry without adding runtime sorting.

The search is intentionally bounded. Both walkers stay within a small square
around the destination tile and the recursion depth is limited. Docking is still
fully abstract: it uses crossbar masks, dynamic lease state, and device jump
resolution only; it does not inspect vendor wire names.

If an incremental Basic route cannot extend its committed endpoint, the
incoming source is marked as a Basic deadend and only that last committed hop
is removed. This rule is the same after an exhausted docking attempt and after
ordinary continuation failure; the parent crossbar must select another
angle-prioritized exit.

## Tile Routed Nets

Each tile keeps a list of routed nets that currently use any node of that tile.
The list is an index for local routing decisions; ownership of the concrete
route remains with the sink instance route vector.

When a route is committed, every tile touched by its wire fragments must
reference the route net. When a net is unrouted, all tile references to that net
must be cleared.

Each net also indexes physical route bindings by complete endpoint identity:
source instance and pin, sink instance and pin, and route identity. Appending a
new branch updates this index in expected constant time; erasing or retargeting
a binding invalidates and lazily rebuilds it. Duplicate endpoint identities are
counted and remain available to consistency-repair code. This avoids quadratic
binding scans on distributed or high-fanout nets without weakening physical
ownership checks.

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
- a failed child edge returns to its parent in Basic, Fanout, Moving-sources,
  and Moving-destinations work;
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
- duplicate preemption suppression expires at pass boundaries while blocker
  ancestry remains available to reject reciprocal preemption cycles;
- busy transit exits remain visible to preemption candidate selection;
- Basic keeps preempted secondary siblings deferred as Fanout work;
- failed Fanout branches advance their branch-point rotation exactly once.

The suite also runs 64 randomized variants each for direct transit takeoff
preemption, joint-mediated preemption metadata, and preference for a free joint
exit over preempting another route.

### `fpga.grounding_preemption` - `grounding_preemption.cpp`

This suite isolates final-entry ownership and proves that grounding:

- does not preempt while any physically reachable complete terminal path is free;
- does not mistake a free destination with an occupied required joint for a
  usable terminal path;
- requires every leased destination and joint on the selected terminal path to
  have an eligible transit owner;
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
- each placement receives bounded routing passes while the focus remains atomic
  across repeated relocations;
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
MUX input lanes. A 512-sink case verifies linear bulk fanout transfer, stable
physical-endpoint reuse, and that topology preparation leaves atomic source-tree
unrouting to the scheduler.

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
- dead terminal seeds release unused beam capacity so a valid later
  predecessor of the viable seed is still expanded;
- a late viable terminal seed retains a breadth-layer slot after an earlier
  seed produces enough dead alternatives to fill the beam;
- docking ignores persistent Basic deadends;
- docking steps out of a destination tile whose current arrival cannot reach
  the required local;
- edge endpoints use the larger configured docking window;
- backward expansion uses the resolved destination namespace;
- only a physically reachable blocked terminal is reported for preemption;
- forward and backward frontiers remain independently observable, and a single
  occupied transit edge between them is reported with its exact numeric nodes;
- an occupied bridge directly at the committed forward anchor is resolved and
  reported when its landing belongs to the backward frontier;
- 20 randomized cases build valid multi-hop backward routes;
- failed backward positions are memoized within one attempt and the reverse
  mapping window is indexed only once.

The routing-state suite also verifies exact bridge suffix removal: the victim's
source takeoff and landing remain leased, all victim nodes at and after the cut
are released, and a sibling branch sharing the takeoff remains unchanged.

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

The `fpga.routing` suite also verifies stage progress accounting without wall
clock sleeps: sub-one-percent windows accumulate, a qualifying window resets
the streak, and a single long search accounts for every elapsed stagnant
minute.

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

### `puzzle.routing*` - `routing_puzzle.cpp`

The routing puzzle builds a deterministic vendor-neutral mesh, first installs a
known complete routing picture, clears every dynamic lease and route fragment,
and asks the production Basic, Fanout, and Moving entry points to reconstruct
all routes. It audits all eight directions, jump lengths 1/2/4, changing route
directions and lengths, tile lease cleanup, shared fanout-prefix ownership, and
final route completeness.

The CTest cases form a regression ladder. `puzzle.routing_basic` isolates
10,000 independent trunks. `puzzle.routing` adds a 1,000-sink shared fanout
tree. `puzzle.routing_90` raises generated routing-resource fullness to 90%.
`puzzle.routing_50k` scales the device to 100 by 100 tiles and creates a
2,000-sink fanout tree, which exceeds one trunk's branch capacity and requires
newly completed branches to become branch candidates. `puzzle.routing_50k_90`
combines scale and high fullness. `puzzle.routing_fanout_forest` instead merges
40,000 routes into 12,000 independent trees with a deterministic skew from
tiny local fanouts to less frequent regional fanouts. It models the many-source
workload where Fanout routing must repeatedly construct and consult thousands
of separate branch frontiers; this is materially different from adding more
sinks to one shared tree. The optional sixth puzzle argument selects the number
of generated fanout trees. New routing regressions should
extend this ladder by changing one pressure at a time and retaining a
deterministic seed, so the first failing level identifies the affected stage or
capacity boundary.

For profiling the fanout workload independently of a vendor database, run:

```sh
./build/routing_puzzle_test 100000 125 100 50 93 13000
```

This creates 13,000 source trees and 80,000 suffix tasks, closely matching the
task shape of the 51,749-cell random-design failure. It is intentionally not a
default CTest because it is a performance investigation case rather than a
short correctness regression.

## Multiple clocks

See [multiple primary clocks](clocks.md) for clock declarations, dedicated
buffer-tree routing and resource-isolation regressions. Asynchronous setup
exclusions do not remove data nets from Basic, Fanouts or Moving.

## Focused congestion ownership audit

Set `SCALEPNR_CONGESTION_NET` to an **exact physical route name** and
`SCALEPNR_CONGESTION_LOG` to a fresh output filename before starting scalepnr.
The default output is `routing_congestion.log`. Unlike the short console
diagnostics, this trace has no event-count truncation. It records every
Generic/Fanout task invocation for that route, before/after fragments and retry
offsets, source and landing rejections, saturated fanout forks, and forward,
backward and terminal Docking blockers. It also applies to those routines when
called by Moving. A new execution is necessary: old unrecorded attempts cannot
be reconstructed after the routing process exits.

Every `BLOCK` has an attempt/event ID and a Tile snapshot (or a reference to an
unchanged snapshot within that search). `SEARCH_ONLY` separates speculative
reservations from live ownership. Snapshots include all live SRC, DST, JOINT,
LOCAL, pin, deadend and incoming masks, packing reservations and binding-index
entries. `PROOF` records identify the actual net, binding, route storage and
fragment, source/sink, coordinates and ownership flags. Shared references do
not count as lease owners. Packing reservations are printed separately and do
not prove a routing lease. An intentionally unleased takeoff LOCAL/output pin
is a `SOURCE_REFERENCE`, not a missing lease: numeric takeoff permits sharing
that source while leasing its outgoing SRC and joints.

The reusable read-only function `fpga::auditTileCongestion(tile, stream,
design_nets)` reconstructs ownership by scanning live route fragments in the
provided design-net scope, independently of the Tile's authoritative binding
index. It reports unowned leases, owned-but-unleased nodes, missing index
registrations and stale registrations. Indexed net pointers absent from the
provided scope are reported without dereferencing them. Callers must supply
all relevant live design nets, including generated nets. It does not repair or
release anything; diagnostic output must not change routing choices.

This is an expensive diagnostic for selected routes, not normal routing work.
Use a saved placement to avoid repeating placement when investigating routing.
Keep the process log beside the trace to distinguish completed attempts from
an attempt interrupted by a timeout.

For ownership changes rather than rejected search edges, set
`SCALEPNR_ROUTE_HISTORY_NET` to the exact RTL net name and
`SCALEPNR_ROUTE_HISTORY_LOG` to a fresh output filename (default
`routing_node_history.log`). This prints complete before/after route trees at
Generic/Fanout task boundaries and route attachment, registration, prefix
promotion, truncation and removal routines. Each scope names the operation and
the calling routing task, including a different net requesting preemption.
Every fragment includes shared/ownership flags and live source, destination,
landing and joint lease bits. Nested scopes have matching IDs. The read-only
serializer `fpga::dumpNetRouteHistory(net, stream)` is also directly callable.
Optionally set `SCALEPNR_ROUTE_HISTORY_NODE=32,55,DST,819` to print every owner
lookup for that numeric node, including the returned binding and an independent
check of its owning fragment flags. SRC, DST, JOINT and LOCAL are supported.
This is a net-tree history, not an instruction-level memory watchpoint; task
boundaries cover direct edits made outside the instrumented mutation routines.
