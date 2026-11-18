!!! Never put in these documents any results of tests or their lists/descriptions or results of experiments or unsure things or general talks. These files only for short requirements / explanations !!!
!!! This is the main requirements file, not history or protocol of changes, be laconic, write worthy things, feel free to reasonably shorten this file or statements here !!!

# Routing Rules

## In brief

### Routing stages, in order

1. **Clock:** connect clock sinks through dedicated resources; protect the completed clock trees.
2. **Const:** connect each constant load from its local constant root; do not treat constants as ordinary trunks or fanouts.
3. **Basic (Generic):** route one complete trunk per ordinary physical driver; leave additional sink branches for Fanouts.
4. **Moving sources:** trace backward toward an unfinished trunk's source, find a legal new source placement, and recursively route its inputs. Finish all trunks before Fanouts.
5. **Fanouts:** connect the remaining sinks by branching from source-connected trunks or completed sibling branches.
6. **Moving destinations (Fanouts moving):** trace forward from an existing branch point, find a legal new sink placement, and recursively route its inputs through Source Moving.

### Tracing (angle-guided) and routing (full search)

**Angle-guided tracing (direction-driven routing):** try the target direction first, then nearby angles to go around obstacles; prefer short hops at the same angle.
Select each next hop with precomputed array ordering, numeric masks and first-set-bit operations, not runtime sorting or candidate scans. Selection is O(1) for fixed-size masks, not for the whole route.

**Recursive hop iteration:** each angle-guided frame represents one Tile/CB hop and keeps its next untried choice in local variables. After a child fails, resume that cursor rather than restarting explored paths.
Use direct `dst_src`, `local_src` and `joint_src` masks with `connected & ~(occupied | deadends)` for transit; retain the destination/eligible Moving endpoint exceptions described below.
Each LOCAL or JOINT cursor step pops one available bit and takes that node's direct exit mask. It does not walk an internal chain or create another recursive frame. Recurse only after a resolved physical jump; backward tracing uses inverse masks and the direction from destination toward source.
Angle-guided tracing may revisit a Tile; it has no Tile-visit guard or extra distance-improvement acceptance rule.
Runtime vector enumeration belongs to `FullSearch`, accessed through Docking. Longer internal terminal chains belong to Docking's bounded full search. Database loading and its stored vectors are separate and unchanged.

**Full search (combinatorial routing):** explore usable alternatives within explicit limits instead of primarily following the target angle.
For ordinary routes and Moving, use it only in bounded Grounding/Takeoff helpers, not chip-wide input proofs. Clock and local Const routing have their own combinatorial engines; their search paths and Docking's own candidate paths allow at most two visits to a CB. An angle-guided prefix is outside Docking's visit count.

**Moving.** Source moving is trying to achieve a new, routable source cell place from destination. Destination movng is trying to achieve a new, routable destination cell place from source.
Select a legal cell packing position along the trace before trying its other inputs; do not run broad input/output routability searches to approve a position.
Then recursively route or move the input drivers. If an input fails, try another parent position; retain successful nested moves and recheck affected connections.

**Deadends:** each Tile has persistent, symmetric `CBState::src_deadend` and `CBState::dst_deadend` masks, separate from occupancy.
Only `TraceRouteForward` writes SRC marks; only `TraceRouteBackward` writes DST marks. A mark requires every eligible SRC in each next Tile, or every eligible DST in each predecessor Tile for reverse tracing, to be persistently marked already. An empty eligible mask also qualifies. Occupancy blocks a hop during the current search, but leases can be released when a recursive branch returns or by later preemption; occupancy alone cannot prove a deadend that must never be rolled back.
All forward searches, including full Docking and repair, filter SRC deadends. All reverse searches, including full reverse Takeoff and Grounding, filter DST deadends, never SRC marks.
A marked jump may be used when its next Tile is the destination routing CB and the connection finishes there without another jump. A resource Tile and its mapped CB refer to that same destination.
Source and Destination moving keep a free next hop for transit even when its Tile cannot host the cell; extend the prefix/suffix without a placement attempt there. Skip an occupied or deadend exit when its next-hop Tile cannot host the cell. An eligible next-hop Tile permits an endpoint attempt, subject to normal ownership and preemption checks. Reverse Moving applies the same rule to DST exits.
Completed-path validation checks ownership and reservations without rechecking deadends. Lease restoration, unrouting, and preemption dont remove deadend masks.

**Packing cluster:** cells that must stay together because of dedicated connectivity. A proposed position must satisfy resource, connectivity and shared-control rules.
`routeOutTry()` checks local output takeoff availability, not complete routes to downstream cells.

### Fabric and connectivity

**Tile / mesh / CB:** a Tile is a grid location with resources and associated crossbar routing; the mesh is their grid. A crossbar (CB) connects routing tracks and local resources.
Resource positions and routing-node positions are different identities; a cell pin maps to a local routing node.

**LOCAL, JOINT, SRC, DST:** LOCAL connects to local resources or a loaded local relay; JOINT is an internal connecting node; SRC leaves a crossbar; DST receives a jump.
SRC/DST describe fabric roles, not the logical driver and sink. A terminal input LOCAL must not be treated as a jump.

**Masks and jumps:** bitmask tables describe legal typed connections inside a CB, directly or through joints/locals.
After selecting a SRC, `dst_by_src` resolves its numeric destination and grid displacement; the reverse incoming index lets backward tracing find its predecessors.

**Crossbar loading and type classification:** read database connectivity, compare configurations and group equal ones into shared `CBType` objects rather than copying a configuration into every Tile.
Different numeric connectivity needs a different type/subtype; equal immutable tables and annotations can still share storage. Each Tile keeps its own mutable `CBState` occupancy and deadends.

**Node names:** annotations for pin associations, diagnostics and export, not strings to search during crossbar routing.
Numeric masks and resolved transitions determine whether a connection exists.

**Sparse routing Tile:** a location with a poorly connected crossbar neighborhood. Packing limits each resource type there to `SPARSE_ROUTING_TILE_LOAD_MAX`, normally 50%.

**Direct resource route:** a dedicated or local pin connection that does not require an ordinary crossbar jump; its endpoint identity still belongs in the route.
**Physical fork:** an extra tap on a fixed inter-Tile wire, distinct from a logical fanout; this model intentionally omits additional physical taps.

### Routes and ownership

**Driver / sink / binding:** the driver is an output pin; the sink is an input pin. A physical binding identifies those endpoints and their particular routed connection, even when logical net names are aliases.
Routes are stored driver-to-sink, including routes discovered by backward tracing.

**Trunk / fanout / sibling:** the trunk is the first routed connection from a driver; a fanout adds another sink; siblings share that driver and existing tree.
**Anchor / branch point:** a source-connected node where tracing starts or a new suffix can fork. A disconnected island is not an anchor.

**Prefix / suffix / fragment:** a prefix is the source-connected beginning; a suffix is the continuation toward a sink; a fragment records part of the physical route and its node usage.
**GAP / island:** a GAP records missing connectivity after a cut without owning resources; an island is retained routing beyond that gap.

**Lease / owner / commit:** a lease reserves a numeric resource for a route owner; committing records accepted routing and its ownership.
Shared routes may have several dependent owners. Speculative paths must not leave live occupation, and a busy bit must have a real owner.

**Unrouting:** remove the selected route or private branch and release only resources no surviving route needs; keep persistent deadends.
Moving a driver invalidates its output trees; moving a sink invalidates its private input suffixes, not unrelated trunks or siblings.

### Endpoints, docking and preemption

**Takeoff:** leave a driver LOCAL, or extend an existing source-connected branch point, into routing fabric.
**Grounding:** connect an arriving route to the sink's mapped input LOCAL. Reaching the Tile alone is not completion.

**Docking:** join a forward route frontier with a backward frontier from the sink using bounded local exploration.
Ordinary Grounding uses a radius-five square; I/O docking uses radius twelve. A frontier is the current end of one direction's exploration, not necessarily committed routing.

**Transit / preemption:** transit passes through fabric without using that segment as its protected start or finish. Preemption cuts eligible foreign transit to make a required endpoint connection possible.
Try free alternatives first; protect takeoffs, groundings and the current source tree, validate the complete replacement and all affected owners, then apply the cut atomically.

**Takeoff / Grounding preemption:** Takeoff can displace eligible transit blocking its exit; Grounding can displace eligible transit blocking required ingress when no free completion works.
Neither permits ordinary transit to evict other transit merely to advance.

**Repair:** reconnect a route broken by preemption, preserving its unaffected prefix and islands. Use ordinary angle-guided tracing, with bounded endpoint docking where needed.
Displaced bindings remain pending work; already completed bindings are skipped when encountered again.

`Wire.cpp` contains route-vector operations. Ownership, lease release, cut
validation, repair coordination and Takeoff retry belong in `RoutePreemption.cpp`,
including their successor traversal, worklists and reachability tables.
Crossbar provides angle-priority ordering.

### Work and failure

**Run / stage / pass / task:** a run admits part of the design; stages execute in the order above; passes belong inside a stage; a task must complete one physical connection.
Recursive work can complete queued tasks, so dispatch checks live completion rather than trusting an old queue entry.

**Soft / hard failure:** exhausting one routing task immediately ends its stage. Basic and Fanouts hand unfinished work to their designated Moving stage; Clock, Const and Moving fail routing outright.
Rejected choices inside a still-active task are not task failures. Failure reports preserve live state and show the failed path, node ownership and a highlighted routing PNG.

The global execution contract is [run-based PnR](pnr.md). Routing must
finish the current admitted part before the next run. Earlier runs' placements
remain fixed during placing, but Moving may relocate their logic
cells in both forward destination tracing and backward source tracing. This
includes cells marked `outline.fixed` and `pnr_completed`; physical I/O sites
remain constrained. Moving selects a packable location along angle-guided
tracing, then routes the moved cell's inputs by recursively moving their
sources. It does not prove input routability with a separate broad search
before selecting the location.
A committed move clears `pnr_completed`, updates the
placement target, and repairs affected ordinary and protected clock routes
before completing the run. Earlier routes retain their ownership by default, but
`pnr_preserved` does not forbid validated transit preemption and repair by a
later run. New fanouts may extend those trees. Infrastructure and endpoint
protections still apply, and every affected connection must be complete
before the current run succeeds.

Numeric crossbar masks and resolved numeric jumps define legal connectivity.
Node names and connection names annotate diagnostics and exported fragments;
missing textual connection annotations must not veto a valid numeric transit.
`fpga.numeric_transit_annotations` exercises named resources without textual
connection records across three physical jumps, beyond immediate grounding.

## Diagnostic Log Retention

When `SCALEPNR_PROFILE_TASK_NET` selects a connection, its brief connection
phase records also go directly to stdout so later history eviction cannot
erase its expansion and accepted-fragment counts.

For a slow individual task, set `SCALEPNR_PROFILE_TASK_NET` to its name or a
distinctive substring. `TASK_TIME_BEGIN` / `TASK_TIME_END` records cover that
task and its nested input routing, with parent IDs, inclusive/exclusive times,
candidate coordinates, anchor, cache result and search counters. These emit
one pair per operation, not one record per expanded node. Candidate rejection
records describe the failed input without enabling full tile-state tracing.
`fpga.task_timing` checks selection, nested attribution, scope restoration and
constant record count for a large simulated search.

## Main routing method

**99% of routing should be done using angle-priority route tracing with recursive calls each containing iterator++

**During angle-priority tracing iterating next hop should be selected by simple mask AND-XOR, no cycles !!!

**All routing tracing must iterate candidate nodes recursively through the
space of possible routes.** This applies to every routing stage, direction,
helper, bounded Docking search and Moving trace. Each recursion frame retains its
current node and next candidate. Descending explores that candidate's
continuations; returning resumes the parent's next candidate rather than
restarting its already explored alternatives.

Optimizations, filtering, direction guidance, caches and hints may accelerate
this traversal, but must preserve its depth-wise node iteration and general
search algorithm. Do not replace it with repeated searches from the roots at
increasing depth, or repeatedly reconstruct and re-expand explored prefixes
to emulate another traversal order. A suspended search must retain the
continuation needed to resume its node iteration. Memory or performance
optimizations must not silently change this contract.

Source and Destination Moving use direction-driven traces, not broad route
searches. Their recursion over input cells is a dependency traversal: each
nested call tries a new placement and tries to route that cell's inputs. Only bounded
grounding/takeoff helpers perform combinatorial wire search. They retain their
own search limits; those limits are not input-proof budgets for Moving.

Repair after preemption follows the same direction-driven rule: select numeric
exits with the ordinary angle priority and prefer short hops at the same angle.
Retaining a GAP or a downstream island does not authorize a separate FIFO,
bidirectional flood or cost-ranked whole-fabric engine. Only the bounded
Grounding/Takeoff part of a repair may use combinatorial search. Preserve the
typed connections and retained owners while changing traversal order.

Direction-led tracing must also follow the loaded typed connectivity inside a
crossbar. A DST may reach a SRC through one or more LOCAL nodes, and a resolved
LOCAL link may reach another crossbar. These are numeric graph edges, not
permission to treat a terminal pin as a jump. Moving's current-placement
completion uses the same typed trace after the hop-only attempt fails, accepts
only the task's mapped input pin, and retains source-connected prefix ownership
until a complete suffix can be committed. It does not enable the general
numeric route-search fallback used outside Moving.

**If one routing task fails to route, it is a soft or hard routing-failure
event and unconditionally ends the current routing stage.** Call
`failRouting()` immediately; do not continue with another task, retry the
failed task in another pass of that stage, or wait for watchdog or timeout.
Soft failure permits handoff to the designated routing stage, never
continuation of the failed stage. Hard failure terminates routing. Rejected
candidates within an ongoing task's recursive search are not task failures.
Failure state is retained by the router: a subsequent batch or direct task
call in the same stage returns failure without attempting work or changing
queues or leases. Resetting that stage's timer cannot resume it. Only an
explicit scheduler transition to the designated next stage, or a new run,
clears this state. Preemption victims that have not failed a routing attempt
remain legitimate pending work; they are not the task that just failed.

## Routing failure

**Report failures without losing unfinished work.** Basic and Fanouts use
soft failure: `failRouting(..., hard_failure=false)` writes the same live
tile state, ownership audit, backward report and failed-net PNG, then returns.
The **first failed connection ends the current stage immediately**. The same
unfinished connection stays first and enters Moving sources (after Basic) or
Moving destinations (after Fanouts) before any other connection is attempted.
It is not counted as completed, rotated behind other work, or left for a later
pass of the failed stage. Routing either completes it or fails routing.
Moving sources (Basic moving), Moving destinations (Fanouts moving), Clock,
and Const use hard failure: diagnostics are written, then routing exits or
throws for the incremental-run caller. Invalid task data remains a hard error.
Partial growth, a leased takeoff, a changed placement, and a `retryable` flag
are not successful task completion.

Search alternatives, backtracking, Docking, and candidate placement previews
are internal to that task. A rejected candidate need not abort the search, but
a failed Moving task must not be deferred to an imaginary later routing
stage. Unsuccessful speculative suffixes must not be committed as
live routing congestion. Takeoffs and the current task's own source tree remain
protected; a foreign shared transit segment may be preempted only by cutting
all and only its dependent paths after full replacement validation. Scheduling
not-yet-attempted tasks and repairing preempted victims must preserve every
unfinished task. Soft reports do not unroute anything or mutate ownership.
Basic/Fanouts deadline and watchdog cancellation end their current pass and
permit routing handoff; Moving deadlines remain fatal. The report includes
`failure_mode=soft|hard`. A run still succeeds only when all admitted work
is complete. This policy supersedes the former all-stages fail-fast rule.

Regression `fpga.routing_fail_fast` covers both Basic/Fanouts soft returns
and hard failures in both Moving stages, Const, and Generic/Fanout batches
executed inside Moving. It checks task retention, completion counters,
unchanged leases on failure, and PNG and text artifacts for both modes.
`fpga.routing_soft_failure_handoff` gives Basic and Fanouts a blocked first
connection followed by a routable connection. Both must attempt exactly one
connection, return failed-stage status, retain the failed connection first,
and leave the second connection untouched. A separate positive control proves
the untouched connection really is routable. Stage-budget regressions check timeout
and watchdog handoff without weakening terminal Moving failures.
`fpga.routing_soft_failure_scheduler_heartbeat` and
`fpga.routing_soft_failure_scheduler_batch` run the full Basic/Fanouts/Moving
scheduler with an unreachable suffix followed by an independently routable
suffix. The first must reach terminal Moving failure while the second remains
untouched. This checks both scheduler loops, not just a batch return value.
`fpga.fanout_search_budget_blocked` and `fpga.fanout_soft_failure_batch`
exercise full scheduler handoff with heartbeat logging enabled and disabled:
an impossible suffix must reach Moving destinations, where its fixed endpoint
causes a hard failure without damaging the completed trunk.

## Shared Crossbar Data

Crossbar configurations read from the device database are compared and grouped
into types. Tiles with the same configuration reference the same type instead
of storing a separate copy of the crossbar configuration for every Tile.
Differences in numeric connectivity, including resolved jump destinations,
require distinct types or subtypes.

Distinct crossbar subtypes keep distinct numeric routing mappings, but share
equal immutable bitmask tables, resolved transitions, and name/connection
annotation containers. Subtype copies initially share these containers.
Database preparation detaches only a container being modified, then interns
equal results using insertion-order-independent content hashes and full
equality checks. Hash collisions never merge different tables. The temporary
interning index holds weak references and is destroyed after preparation;
crossbar types own the shared data for as long as they need it.

Routing uses read-only `get()` lookups on these tables, including missing-node
lookups, without allocating entries or detaching shared storage. No hashing,
text matching, or table interning is added to routing searches. Dynamic leases
and deadend masks remain independent in each tile's `CBState`.

## Sparse Routing Neighborhoods And Placement

Database loading builds a numeric crossbar-presence map after assigning the
physical crossbar owners and resolving their subtypes. For this capacity map,
a switchbox must offer at least two distinct outgoing SRCs from one incoming
DST, taking the union of direct exits and exits reachable through joints.
One-to-one transit bridges and local-only fabrics do not count as neighboring
switching capacity. Their routing connections remain available; this rule
only controls packing quotas. Multiple paths to the same SRC count once.
An attached resource view of a crossbar does not count a second time.

`fpga.sparse_bridge_capacity` uses neutral node names to check absent neighbors,
14-lane one-to-one bridges, and branching switchboxes in mirrored layouts.
It checks sparse flags, preview rollback, and actual packing admission: adding
a bridge must preserve the 50% quota, while a switchbox can restore full capacity.

A physical crossbar is **sparse** when `(top absent OR bottom absent) AND
(left absent OR right absent)`, using immediately adjacent mesh coordinates.
The horizontal condition means that **both neighbors are not present**, not
that both must be absent. Outside-device neighbors count as absent. Missing
only one horizontal crossbar is normal for layouts with a resource column
beside the switchbox; it is not sufficient to classify the crossbar as sparse
without also missing a vertical neighbor. `Tile::sparse` is
then copied to the resource tiles sharing that crossbar, including resources
attached on its left or right. The flags are derived from the device database,
not route occupancy, tile names, or design-file annotations.

`fpga::SPARSE_ROUTING_TILE_LOAD_MAX` in `Tile.h` sets the maximum per-resource
occupancy percentage, initially **50**. Each sparse tile may occupy at most
`floor(number_of_positions * SPARSE_ROUTING_TILE_LOAD_MAX / 100)` positions
of each element type independently. Thus eight positions allow four occupied
positions, sixteen allow eight, two allow one, and a singleton allows none.
This is an occupancy quota, not a removal of position bits: any lane remains
eligible if packing connectivity permits it. A full LUT also consumes its
paired secondary LUT resource quota.

The same limit applies to placement, exact-position packing, temporary packing
previews, generated passthrough cells, and routing-time moves. Pre-smear,
predicted-capacity and sorting estimates use the reduced capacities too.
Unassigning a cell or rolling back a preview frees its quota immediately.
Existing placement caches must be regenerated to evaluate this policy; simply
loading old placements does not repack them into the new limits.

`fpga.packing` tests all sixteen cardinal-neighbor combinations, physical
crossbar versus attached-resource identity, device edges, map regeneration,
independent quotas for every modeled element type, high-numbered positions,
exact/ordinary/preview packing, preview rollback, unassignment, singleton
rounding, and paired LUT resource consumption. Tests use synthetic topology
and neutral resource names, without a vendor database.

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

Basic routing is direction-driven with immediate combinatorial Docking attempts. It
performs an initial, brief allocation of routing resources for
every net. For a multi-fanout net, it must select and route only one initial
trunk to completion; the not-yet-attempted sink branches are deferred to Fanout
routing. An incomplete trunk is fatal at its task boundary, not successful
Takeoff allocation and not work to hand to Moving sources.

Every forward routing search reads `CBState::src_deadend`; every reverse
search reads `CBState::dst_deadend`. Docking and repair use the same directional
checks as the traces. Only `TraceRouteForward` and `TraceRouteBackward` write
their respective masks, after the next Tile's entire directional exit mask
is occupied or deadend. A failed individual choice, cancellation, or budget
limit cannot substitute for that condition. A direct destination landing is
exempt because it does not require another jump. Both masks survive unrouting.
Moving may traverse a free hop whose Tile cannot host the cell, without a
placement probe there. An occupied or marked hop is considered only when its
next Tile permits the endpoint placement; occupancy still requires valid
ownership reuse or preemption. Reverse Moving applies this rule to DST marks.

Grounding is the final search step for a Basic route. It uses the special
Docking algorithm and performs a full
combinational search in both directions within a square of radius five Tiles. One search
frontier grows forward from the routed prefix and the other grows backward from
the destination. Reverse passes filter DST marks; forward passes filter SRC marks.
Both permit a direct destination landing without another jump.

If the destination is already within five Tiles of the source (the existing
Docking metric, `max(abs(dx), abs(dy)) <= 5`), try Grounding before ordinary
forward routing. Its backward search starts at the destination input LOCAL
and may finish at the exact source output LOCAL, including its Takeoff and
any intervening JOINTs. LOCAL and DST identifiers are separate namespaces.
The source output also seeds Docking's forward half through its legal
`LOCAL -> SRC` Takeoff transitions. Do not insert the LOCAL itself into a DST
frontier or disable this forward half: on dense CrossBars, backward-only
breadth-first search can enumerate thousands of states for a two-hop route.
Both halves remain speculative, and backward search can still reach the
source when its Tile lies outside the forward window. No narrower breadth
limit is used to obtain this speedup.
A successful short-route Grounding skips ordinary forward routing; a failed attempt
leaves leases unchanged and allows the normal forward search to continue.

Within a tile, Grounding (`DST` to local) and Takeoff (local to `SRC`) may
preempt a transit route that occupies a required resource. A preempted victim
must be unrouted and queued for routing again. The safe default is to unroute
the complete victim route; an implementation may preserve a prefix or other
part only when that part is ownership-separable and retaining it cannot leave
stale leases or an invalid route tree.

Basic does not hand failed attempts to another stage. Completed trunks stay
leased; preempted or otherwise invalidated routes are explicit repair work.
Every selected repair task follows the same finish-or-fail boundary.

Basic throughput must be measured separately from device loading and later
stages. `SCALEPNR_ROUTE_PROFILE=1` with `SCALEPNR_ROUTE_PASS_DETAIL=1` prints
per-pass timings. `priority_ms` measures numeric priority-list construction and
lookup; `conflict_ms` measures takeoff victim lookup. Both are included in
`bf_loop_ms`, so these counters must not be added as independent costs.
`SCALEPNR_DEBUG_STOP_BASIC_PASS` and `SCALEPNR_DEBUG_BASIC_COMPACT=1` allow a
bounded, reproducible comparison from the same saved placement.

Passthrough net appends extend the numeric designator index instead of rebuilding
it over the entire module. The index stores offsets and resets at each routing
transaction. Priority construction skips empty fallback masks and uses single-bit
access without changing angle/length/lane order. Takeoff preemption first checks
the exact tile/SRC fragment, then applies source-tree protection to its owner;
unrelated tile routes need no source-identity lookup. These optimizations do not
change routing priorities, search limits, or victim selection.

### 4. Moving Sources

**Engine: direction-driven backward tracing, followed by recursive cell
movement.** It reads and writes the DST deadend mask through `TraceRouteBackward`. The stage works on
unfinished trunks and must finish with zero unfinished trunks before Fanouts
starts.

1. Select one unfinished physical source-port-to-sink connection. If its live
   binding is already complete, skip it without changing placement or leases.
2. Trace backward from the sink's mapped input through the numeric incoming
   index. Prioritize the angle toward the old source, then shorter hops within
   the same direction. Use existing numeric mappings, not wire-name lookups.
   A rejected packing location does not reject the routing node: continue
   tracing from it. Direction-driven tracing does not prohibit revisiting a
   Tile through another routing node. A `Docking` object supplies the driver
   placement probe, stop check, and optional state view to this backward trace.
   Intra-crossbar transit may use a free `DST -> LOCAL -> SRC` relay,
   optionally through joints after the LOCAL. Preserve those typed arcs;
   a relay is not a cell input. Pin leases and packing reservations still
   forbid using that LOCAL as transit.
   Start at the mapped input LOCAL, not only its compact DST terminal table.
   Preserve longer JOINT chains and resolved cross-tile LOCAL connections as
   typed numeric edges. An omitted compact terminal entry is not evidence
   that the pin has no free ingress. Temporary tracing leases are restored
   on success, failure, and cancellation before any real move is committed.
   Exhaust the free traces before considering a blocked incoming edge. Keep
   the exact numeric SRC/DST obstruction, validate its real transit owners
   through the existing preemption rules, and requeue every displaced binding.
   Retry only after the selected occupied bits were actually released; rebuild
   the temporary ownership view from the new live state. Protected endpoints
   and disconnected LOCAL bridges do not authorize a cut.
   A failed child resumes its parent's next incoming source or local transit
   choice. Each frame retains its direction, source and node cursors; returning
   to a parent must not repeat its earlier placement probes. Keep all blocked
   arrivals encountered during free tracing, because a protected first owner
   must not hide another eligible transit edge.
3. At a reached crossbar, test legal resource positions for the source's strict
   packing cluster. Packing includes connectivity and shared control-pin
   compatibility. `routeOutTry()` checks local takeoff availability for the
   connected outputs, including the selected trunk's exact SRC. It must not
   launch an input-route proof or a chip-wide output search.
4. Select the placement attempt before routing its other inputs. Retire routes
   whose endpoints change: moving a physical driver invalidates its source
   trees; moving an input invalidates its private suffix, preserving shared
   trunks and surviving branches. Move the cluster and commit the prepared
   trunk in source-to-sink order.
5. Reconstruct every live ordinary input, including inputs without a stored
   route binding. `moveInputSources()` first skips completed bindings and can
   try normal angle-guided routing at the existing source location. For each
   remaining input it calls the same `moveUnfinishedSource()` method, which
   may move that driver and recursively process its own inputs.
6. Return success only after all of the moved cluster's inputs are complete.
   Recheck earlier inputs after a nested move because it may invalidate another
   connection. Preserve pending displaced branches and other affected work;
   never lose a task because recursion completed or invalidated its queued copy.
   If recursive input routing rejects this placement, remember the attempted
   parent position, release its output tree, and continue Source Moving at a
   different position. Successful nested moves remain committed. This is a
   new placement attempt against changed live ownership, not an input preflight
   search or a restart of an unchanged wire search. Exhausting the remaining
   positions fails the task and retains the failing input's identity.

### 5. Fanout Routing

Fanout routing is direction-driven with immediate combinatorial Grounding/Docking
attempts. It starts only after Moving sources reaches zero and routes every
deferred suffix of each multi-fanout net from the trunk state established by
Basic or repaired by Moving sources. Fanout routing uses the same persistent
SRC deadend mask. It may still preempt a transit route when that route blocks Grounding,
subject to the same victim-safety and requeue requirements.

Fanout search advances through downstream arrivals on the trunk and then
completed sibling branches. Prefer Tiles with fewer existing private forks;
reuse earlier Tiles when necessary, rather than clustering suffixes in one CB.
Fork usage is recovered from completed routes when the branch index is rebuilt.
The source Tile is a last-resort fallback: the already connected driver output
may drive another free Takeoff without removing its existing trunk, but must
not consume a new Takeoff while an earlier downstream attempt can succeed.
This includes a complete local-only trunk: its source-position fragment can
have no SRC jump (`jump == -1`) while its driver LOCAL is still a valid start
for a remote suffix. Do not require an existing inter-Tile hop to discover
that source. Keep the local trunk and its input lease intact.
Repairing a preempted route must also consider a replacement Takeoff from its
real driver when the retained prefix is trapped outside the Grounding window.
Try free paths first. A new Takeoff may provisionally displace eligible transit
at its source and first landing; intermediate LOCAL bridges do not grant new
Takeoff rights. Subsequent transit must be free until bounded Grounding.
Validate the complete replacement and combined cut footprint before changing
any lease, retain unaffected victim sections, and requeue affected bindings.
A reached destination Tile must be
tested for Grounding even when it has no free outgoing SRC: completing at its
input does not require leaving that Tile. Existing same-signal prefixes remain
shared; new extensions must not steal resources from other signals.
Failure to complete the selected
suffix calls `failRouting()` immediately; it must not leave a leased partial
suffix for Moving. A trunk invalidated by
Fanout preemption is
repaired as Generic work inside the Fanouts stage without re-entering Basic or
changing the shared-mask policy.

### 6. Moving Destinations

**Engine: direction-driven forward tracing, followed by the same recursive
Source Moving used above.** It operates on unfinished suffixes and uses the
same persistent SRC deadend mask as other forward searches.

1. Skip a task whose live physical binding is already complete. Start the
   unfinished suffix at a valid source-connected branch point in its trunk or
   an existing routed sibling. Never use a disconnected route island as an
   anchor or unroute the trunk merely to create a new branch point.
2. Trace forward with angle priority toward the old destination and short-hop
   priority within that direction. At each reachable next hop crossbar, test packing and
   the triggering input's connection from the actual reached DST.
3. Once the candidate is selected, move the destination's strict packing
   cluster and commit the triggering grounded suffix. Invalidate all private
   input suffixes whose sink position changed, not the unrelated shared trunks
   or sibling branches. Invalidate output trees only where their physical
   driver actually moved.
4. Call `moveInputSources()` on the moved cluster's inputs. It uses the same
   `moveUnfinishedSource()` recursively for their upstream cells; Destination
   Moving has no separate input-reachability or input-budget algorithm.
   If an actual input attempt fails, retain successful nested moves and try
   the next parent position in the existing tried-position sequence. Rebuild
   candidate state from live placement and ownership; do not restore an old
   snapshot over a child's committed work. Exhausting parent positions reports
   the failing input as well as the final placement failure.
   Once recursive inputs reject a parent position, completing only the trigger
   at that position cannot turn the rejected move into success. The in-place
   completion shortcut must not bypass the remaining input requirements.
5. Recheck incident completion after nested changes and keep all displaced
   tasks accounted for. Outer task lists consult the current binding and skip
   inputs already completed recursively. A successful placement alone is not
   a successful move if one of its required inputs remains unfinished.

## Constant Routing

Logical constant inputs remain attached to the design's global constant
connections. A crossbar database may declare continuously driven local nodes in
`constant_one_nodes` and `constant_zero_nodes`. `RouteVCC` discovers those
logical loads, creates stable logical source/net identities, and prepares any
required tile-local passthrough endpoints. It does not search paths or lease
nodes.

Moving must collect the physical distributed-constant binding as an incident
route, not reconstruct a generic task from a literal alias. The logical global
source belongs to the unplaced top instance; it is not a movable driver.
The append-aware net index resolves physical constant nets by
`distributed_source` and `distributed_one`, independently of their names.
This same incident collection drives old-lease invalidation and immediate
constant rerouting after relocation. Unbound constant loads remain owned by
Const-stage preparation, not the generic input-task collector.
`pnr.moving_constant_alias` reproduces the erroneous unplaced-top task and
requires exactly one physical task for an existing binding, for both values.
`pnr.constants` verifies that both values actually route to completion across
incremental runs; it does not merely check task discovery.

Inserting a target passthrough must preserve the global source's designator
(`-1` for zero, `-2` for one). The new passthrough input uses the generated
physical net's positive designator while still following the logical global
connection. Rewriting the global designator changes every unrelated constant
consumer: later incremental placement no longer recognizes their required
input endpoints and can admit conflicting controls on one physical local.
`fpga.passthrough_global_control` reproduces this corruption through production
endpoint insertion, for both polarities and both placed and deferred helpers.
It checks that a later chunk rejects constant/signal sharing in previews and
committed packing, without importing device node names into the fixture.

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

## Clock Routing

Clock routing is the first top-level stage and is implemented by `RouteClocks`.
It is independent from the four ordinary routing stages and from Const routing.
Clock tree search reads the forward SRC mask. Any request to learn a mark
goes through `TraceRouteForward` and its complete next-Tile SRC-mask condition.
A direct destination landing does not need an outgoing jump. Dedicated
LOCAL-only pass-through edges have no SRC to mark.

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

Later placement routing never converts a clock branch into an ordinary route
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

### Grounding Docking

Grounding Docking runs immediately when forward search reaches a Tile/DST
inside the destination window, before that position's ordinary exits are
expanded. It tries free completion first, then permitted transit preemption
with full replacement validation. Success ends the current connection search;
failure resumes ordinary tracing. Revisiting a search-stack frame does not
repeat an already attempted Docking operation for that same candidate prefix.
There is no post-search sweep of saved positions and no single-position
preemption fallback. The search stack itself retains the current prefix and
untried exits, as required for backtracking.

Docking searches from both ends without committing speculative wire fragments.
One side walks backward from the destination local through possible incoming
destination tracks. The other side walks forward from the current route frontier
through possible outgoing source tracks. If the two searches meet, the collected
fragments are committed to the route and leased normally.

The additional provisional-preemption fallback expands **both** directions through
eligible transit obstructions. In particular, a busy first forward exit must
not silently reduce bidirectional Docking to a backward-only search. Both
directions use the same read-only ownership/eligibility predicate. No victim
is cut while growing either frontier: the complete joined path, retained
prefix and combined release footprint must pass validation before the atomic
commit. Genuine Takeoff/Grounding owners remain protected. The free searches
still run before this provisional pass.
For complete Fanouts tasks, all established branch/source attempts run first.
Only after the whole task exhausts them does a second bounded traversal enable
occupied forward growth. This adds a completion opportunity without replacing
an already successful later branch by an earlier preempting path. Within that
fallback, established provisional joins are still preferred. No failed task is
handed to another task, and cancellation prevents starting the fallback.
If no provisional Docking attempt occurred, the fallback is skipped: enabling
forward Docking cannot help a search that never reached a Docking attempt.

When source repair replaces a route that supplied another fanout's shared
prefix, prefix ownership transfer includes its ending DST. That lease may
belong to the departing route's first private TRANSIT fragment rather than
an explicit `owns_landing` flag on the common prefix. Transfer it to the
surviving prefix before releasing the old route; its private FORK must not
be left borrowing an unleased DST.

## Inter-Tile Physical Forks: Intentional Simplification

Routing intentionally ignores additional branches of fixed physical wires
between tile crossbars (CBs). These physical forks are considered rare, and
supporting every additional tap is not worth the added topology and ownership
complexity at present. Keep the supported inter-CB connections without expanding
these forks into extra routing alternatives, accepting the loss of those
connection opportunities.

This limitation concerns physical wire forks, not logical net fanouts: the
Fanout stage must still route every sink. It also does not justify losing
continuity through a wire inside one CB merely because that same wire appears
as both a `SRC` and a `DST` in the model.

## Intra-CB Wire Continuity

When the same physical wire name has both `SRC` and switchable `DST` roles in
one CB, the loader connects those roles with a zero-displacement numeric
continuation. Their numeric IDs may differ; equal IDs with different physical
names do not establish continuity. Tile-specific topology rebuilding must
preserve this connection even when the wire has no inter-tile connection.

Forward and backward routing must traverse these continuations. A same-CB
step has no compass direction and does not count as leaving and revisiting a
tile. Ordinary SRC/DST lease checks still apply, and the route keeps both roles
in its fragments so ownership and unrouting remain explicit. This does not
expand the inter-CB physical forks intentionally omitted above.

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

Regression fixtures live in `src/tests`. Focused tests, including focused
placement tests needed to reproduce a bug, may be run individually. Do not run
the entire standalone placement suite as part of the routing regression gate.
Exclude both `placing.*` and `puzzle.placing*` from ordinary routing regression
runs; select any required focused cases separately and report their results
separately. The bulk generated placement suite under `tests/placing` remains
opt-in. Packing and routing correctness regressions remain included.
Placement performed inside an explicitly requested full PnR run is unaffected.
Use a `gpt-6-sol` test runner to execute tests and return a one-page analysis of
the results and logs before deciding on fixes or further investigation.
