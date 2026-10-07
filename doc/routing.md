# Routing Rules

The global execution contract is [run-based PnR](../docs/pnr.md). Routing must
finish the current admitted part before the next run; earlier runs' placements
and routes remain fixed and owned.

This document records generic routing behavior that must stay independent of
any FPGA vendor database.

## Main Routing Principle

**Finish the current connection or stop routing.** In Basic, Moving sources,
Fanouts, and Moving destinations, a selected task must reach its sink before
another task starts. An exhausted or cancelled task calls the terminal
`failRouting()` routine (the `routing_failed()` failure-reporting path), writes
the failed-net diagnostics/PNG, and exits unsuccessfully. Clock and Const obey
the same rule. Partial growth, a leased takeoff, a changed placement, and a
`retryable` flag are not successful task completion.

Search alternatives, backtracking, Docking, and candidate placement previews
are internal to that task. A rejected candidate need not abort the search, but
a failed task must never be rotated, deferred to another pass/stage, or replaced
by another sink. Unsuccessful speculative suffixes must not be committed as
live routing congestion. Takeoffs and the current task's own source tree remain
protected; a foreign shared transit segment may be preempted only by cutting
all and only its dependent paths after full replacement validation. Scheduling
not-yet-attempted tasks and repairing preempted victims does not authorize
yielding the current task unfinished. This rule supersedes the old partial
prefix/time-sliced recovery policy.

### Endpoint priority and partially routed victims

Only Takeoff/Grounding (including its Docking completion) may preempt foreign
transit resources. Transit may not preempt transit, and endpoint resources
may not be preempted by another endpoint. Eligibility follows physical node
ownership, not a route's name or preemption history. There is no once-per-pass,
once-per-route, or historical route-pair ban; these would incorrectly prevent
independent cuts and repairs on different segments.

Grounding first searches free alternatives. Only if no free completion is
available does it explore through eligible foreign transit obstructions
provisionally. A busy SRC must not hide its free predecessor DST in this
second phase. The search verifies owners without cutting or leasing anything;
it carries the occupied resources of the proposed suffix through subsequent
transitions, including an occupied terminal entry. Before committing, validate
the entire forward prefix and Grounding suffix against the combined releases
of all required victims. Apply those cuts together only for a fully validated
replacement. Failed exploration or validation leaves every victim unchanged.
Before deeper provisional exploration, try complete single-bridge and terminal
replacements already discovered by the free search. During expansion, check
only the newly encountered edge's physical owners; do not reconstruct and
revalidate the whole suffix for each predecessor. Reject duplicate or
over-width expansion candidates before ownership scans, while still allowing
complete frontier joins. A full layer must skip queuing an over-width
candidate, not terminate predecessor enumeration: a later numeric DST may
already complete the connection or expose a complete bridge. Dependent fanout
scans, combined release previews, and full replacement validation belong to
the completed-path check, not the per-edge eligibility check.

Committed paths have three states: **unrouted**, **partially routed**, and
**routed**. A preempted transit path retains both its connected upstream prefix
and leased downstream fanout wiring. An explicit resource-free gap records the
missing connection between exact numeric DST nodes, or from a DST to the
reserved LOCAL input when the affected segment reaches a terminal tail. Downstream branches remain
reserved, but dependent sinks are not reported as fully connected until repair.
Branches that do not cross the cut remain routed.

Preemption removes only the contiguous transit segment needed to release the
conflicting nodes. Reusing a SRC also requires releasing its landing DST.
A direct request for another endpoint's Takeoff/Grounding remains forbidden.
When an eligible upstream transit cut reaches the victim's terminal tail,
retain its LOCAL input lease and reroute only that tail, including its old
DST/JOINT input path. Do not freeze a disconnected Grounding on an unusable
arrival, or reject every last-transit cut simply to retain that arrival.
Unrelated fanouts and other endpoints' packed joint reservations remain protected.
Adjacent cuts merge into one gap.
Before ordinary task continuation, repair searches free fabric between the
retained boundaries, ignoring deadend hints and without further preemption.
It may reuse an exact fragment of the same source tree. Successful replacement
updates all replicas of that gap, retaining their existing downstream routing.
If an exact cut boundary is trapped, try a bounded source-to-sink replacement
from the actual driver, including connected same-source branches. Traverse
numeric LOCAL/DST/JOINT/SRC arcs, including legal intermediate
`DST -> LOCAL -> SRC` routing wires; a LOCAL is not necessarily a cell input.
Foreign leases, packed inputs and existing inputs of retained same-source
nodes remain protected. Validate a complete replacement before releasing any
old resources. Install and register the new path first, then release only
old claims without surviving owners. Other bindings retain their routes.
This fallback uses the repair state/hop limits and cancellation checks; the
optional whole-device reachability audit is diagnostic, not the routing loop.
Intermediate LOCAL wires have ordinary routing leases, not endpoint pin
leases, and remain eligible for transit cuts and subsequent repair.
Failed or cancelled searches do not release the retained islands and propagate
failure to the existing fail-fast task boundary. Partially routed is not a
successful final result and does not authorize committing a failed new suffix.

Implementation boundaries: the normal batch loop, heartbeat loop, and legacy
per-instance entry point use `routeTaskOrFail()`. Source/destination relocation
must report a complete triggering binding before advancing. Clock-tree helpers
return immediately on their first failed sink and the stage caller invokes
`failRouting()`. Const uses the same ordinary task boundary; synchronous
constant repair stops at its first failure and propagates that failure to the
current movement task. None may reinterpret failure as a scheduling yield.

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
and Fanout routing use direction-driven forward search and immediately try
combinatorial Grounding/Docking at each reached position inside its window.
Moving sources, destination-to-source routing,
backward parking against retained anchors, and Moving destinations are
combinatorial. Clock routing is combinatorial tree search. Const routing is a
combinatorial local-graph search confined to one crossbar, so it passes that
crossbar only once by construction.

## Sparse Routing Neighborhoods And Placement

Database loading builds a numeric crossbar-presence map after assigning the
physical crossbar owners and resolving their subtypes. A transit crossbar has
an incoming-to-outgoing path, directly or through joints. Local-only fabrics
do not count as transit switchboxes. An attached resource view of a crossbar
does not count a second time.

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

Only Basic routing may learn and enforce persistent `SRC` deadend marks. A
`SRC` is marked as a deadend when no continuation after that node is free
because every possible continuation is occupied. These marks expose congested
areas and must prevent later Basic searches from entering those areas.

Grounding is the final search step for a Basic route. It uses the special
Docking algorithm, ignores persistent deadend marks, and performs a full
combinational search in both directions within a 5-by-5-tile window. One search
frontier grows forward from the routed prefix and the other grows backward from
the destination.

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

### 4. Moving Sources

Moving sources is combinatorial, including destination-to-source search and
backward parking against retained route anchors. It may repair invalidated
trunks, but is not a fallback after a failed Basic task. It ignores
persistent deadend masks and routes backward from the fixed destination before
changing placement. The reverse search follows the numeric incoming-jump index
until it reaches a nearby route tile where a legal source placement can drive
the selected `SRC`. Only then does it invalidate the old source tree, move the
physical source cell, and commit the already-proven trunk. Other bindings from
that source remain parked suffixes.

Every Moving Sources reverse search is limited to **10 routing hops** by
`movingSourceSearchDepth()`: retained-prefix docking, replacement takeoff
search, and each candidate's input-route proof. The limit counts graph hops,
not Manhattan tile distance. All candidates inside this depth are eligible;
there is no separate placement-probe count limit. Exhausting this bounded
search does not prove that no longer route exists.

When relocation replaces an input with a different proven route, attaching the
replacement must also retire the old retained input prefix. Register the new
route before releasing old leases, preserve shared-prefix ownership in surviving
branches, and clear the old storage. Redirecting the binding alone leaves false
congestion. `fpga.moving` tests fresh-takeoff and shared-prefix replacements,
including sibling survival, stable binding IDs, and clean congestion audits.

An existing partial forward route is the first recovery target. Reverse routing
visits incoming nodes in breadth-first hop order and docks to a reachable
numeric anchor, preserving the source takeoff and useful prefix. It must not
commit a long depth-first detour while a shorter free suffix is waiting in the
frontier. Angle priority orders equal-depth candidates, and duplicate anchors
at the same numeric node retain the newest prefix landing. If no anchor is
reachable, a failed prefix is released before replacement search so it cannot
remain as permanent congestion. This release is per binding and occurs only after every
retained anchor has failed; unrelated prefixes remain leased. A speculative
state view hides only the selected route's own leases while its replacement
trunk and source placement are tested.

The combinatorial reverse walk expands depth layers, preserving angle order
within each layer. All free terminal entries receive a turn before deeper
branches: an early long lane must not hide a short connection to a retained
trunk. This does not change the direction-driven Basic or Fanout engine.
`backward_anchor_search_does_not_starve_free_terminal_entries` in
`docking_test.cpp` checks a one-hop anchor beside a separate long reverse lane,
including preservation of the live leases.

Cancellation does not prove a routing deadend: source recovery must return
without releasing its prefix or preempting a boundary victim after cancellation.
The progress watchdog includes the unfinished work temporarily held inside a
relocation batch, and stage totals include these relocation attempts/completions.

Moving Sources probes reached takeoffs after each complete reverse depth layer,
instead of exhausting the whole chip's reachable component before testing any
replacement placement. Integer distance buckets prefer the old source within
the reached layer, without runtime sorting or an old-placement radius cutoff.
Rejected placements do not authorize preemption while a free continuation
remains queued. Expansion continues and probes each newly reached layer first.
Only after the free frontier and its placement probes are exhausted may the
existing transit-preemption policy release one exact blocked reverse edge.
The bounded numeric blocker sample is retained across these layers.
The search then returns immediately:
its speculative state view must not survive a live ownership change. The normal
scheduler requeues the affected routes and retries with fresh states.
Cancellation or a budget leaving unexamined free work cannot authorize a cut,
including in the caller. A bounded probe advances its offset even when all
currently discovered candidates were examined but deeper free nodes remain.
Fixed-source input and anchor searches in Moving Sources use the same ten-hop
limit; other callers retain their existing policy.
If the existing placement can drive the reached exact takeoff, the
trunk is committed without moving or disturbing its inputs. Otherwise the
proven route is retained and the source moves to the
nearest reached location in that layer whose packed resource and takeoffs are legal. There is
no arbitrary source-distance cutoff that can discard the only proven route.
The backward API supports a bounded probe window with a persistent per-source
cursor, but Moving Sources requests all probes inside its ten-hop search,
subject to stage cancellation. A bounded caller can continue after previously rejected
frontiers instead of rescanning them. When both the
numeric reverse frontier and all of its placement windows are exhausted, the
search reports that exhaustion directly. A successful route clears the probe
cursor.

When a trunk starts at a generated tile-local passthrough, the route endpoint
remains the passthrough but relocation follows its void source chain to the real
physical driver. Generated adapters are never selected as independently movable
source cells. The complete strict packing cluster is previewed with the external
route endpoint fixed at each candidate element lane. A candidate is accepted
only when the preview assigns every physical and generated cluster member.

Source Moving handles each selected trunk atomically. Anchor probing and
placement alternatives are internal search operations. A rejected candidate
may restore placement, routes, leases, and queues before the same search tries
another candidate. Once `moveUnfinishedSource()` returns failure, however,
the top-level stage must report and stop, even if it labels the result
retryable. It must not rotate that trunk behind another source.

A successful relocation must have completed the triggering trunk. Its
required input-routing tasks are synchronous and obey the same fail-fast
boundary. Success alone permits the next source to start. Empty active work
does not imply stage completion while repair queues remain nonempty.
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
Input proofs may dock onto their own complete or incomplete route's fabric
prefix, even when its sink is the cell being moved. A successful proof passes
its exact selected prefix length to bulk sink invalidation: retain that anchor
and its ownership, release the unused private tail and old terminal, and exclude
every route whose driver moves. Incomplete private prefixes without a successful
proof keep the normal invalidation behavior; they are not retained by default.
A sibling anchor must
survive the move; another moved input's replaceable tail is not an anchor.
An input extending its own prefix updates the original binding and keeps its
ownership, releasing only the tail after the selected anchor. It is not a new
shared-only copy. Input validation has no fixed 4096-expansion cutoff: it runs
until it reaches the fixed driver/tree, exhausts the search, or hits the stage
deadline/watchdog. Cancellation is never cached as an impossible placement.
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

Fanout routing is direction-driven with immediate combinatorial Grounding/Docking
attempts. It starts only after Moving sources reaches zero and routes every
deferred suffix of each multi-fanout net from the trunk state established by
Basic or repaired by Moving sources. Fanout routing ignores persistent deadend
masks. It may still preempt a transit route when that route blocks Grounding,
subject to the same victim-safety and requeue requirements.

Fanout search advances through downstream arrivals on the trunk and then
completed sibling branches. Prefer Tiles with fewer existing private forks;
reuse earlier Tiles when necessary, rather than clustering suffixes in one CB.
Fork usage is recovered from completed routes when the branch index is rebuilt.
The source Tile is a last-resort fallback: the already connected driver output
may drive another free Takeoff without removing its existing trunk, but must
not consume a new Takeoff while an earlier downstream attempt can succeed.
A reached destination Tile must be
tested for Grounding even when it has no free outgoing SRC: completing at its
input does not require leaving that Tile. Existing same-signal prefixes remain
shared; new extensions must not steal resources from other signals.
Failure to complete the selected
suffix calls `failRouting()` immediately; it must not leave a leased partial
suffix for Moving. A trunk invalidated by
Fanout preemption is
repaired as Generic work inside the Fanouts stage without re-entering Basic or
consulting Basic deadends.

### 6. Moving Destinations

Moving destinations, also called Fanouts moving, is combinatorial. It is
responsible for repairing suffixes invalidated by routing or movement.
It is not a handoff for a failed Fanout
task. It selects an unrouted, congestion-blocked
destination cell and grows its suffix forward from a source-connected branch.
Every reached tile is tested for legal packing and grounding before the cell
can move there. It ignores persistent deadend masks.

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

The default hard budget is 10 minutes per stage. Independently, a progress
watchdog samples committed outstanding work in one-minute windows. A window
must retire at least `ceil(1% * tasks_at_window_start)` tasks; ten consecutive
deficient windows cancel the current search. Cancellation is terminal for the
current task in every stage, including Basic: it must not hand a failed task
to Moving or advance to another task. A qualifying window
resets the deficient-window streak. Search and relocation cancellation points
poll the same watchdog, so one long speculative operation cannot hide a stalled
stage until its hard deadline. The window and streak are configurable through
`SCALEPNR_ROUTE_PROGRESS_WINDOW` and `SCALEPNR_ROUTE_STAGNANT_WINDOWS`.
On failure, scalepnr overwrites `routing_failure.png` and
`routing_failure.txt` in `SCALEPNR_FAILURE_ARTIFACT_DIR` (or the current
directory when unset). The reported task is selected from live unfinished
queues by the most recent failed-attempt sequence, excluding completed bindings
and tasks marked for retirement. The 5x5 PNG is centered on that attempt's
recorded congestion tile and highlights its failed net and numeric node. Reverse
search retains the last blocked hop and its connected suffix; when no blocked
hop exists it uses the deepest explored frontier. These paths are unleased
diagnostics and never influence routing decisions. The same failed exit prints
the center tile's full SRC, DST, JOINT, LOCAL, pin, and deadend masks to stdout,
followed by its live ownership audit. Retained
failed-search paths may illustrate that task but cannot select a different,
already-completed net. Thus the image and textual diagnosis stay beside the run
that produced them without accumulating stale reports. A new routing run
removes these artifacts before starting, including when that new run succeeds.

Forward continuation also retains the deepest exhausted node of a failed
uncommitted suffix. In particular, filtering every outgoing SRC as occupied
must record that downstream DST, even though no individual lease is attempted.
Backtracking to an exhausted parent must not replace it with the saved Wire
tail. A successful alternate suffix discards this diagnostic; recording it
does not commit or lease any part of the failed search.

`routing_failure_backward.txt`, linked from the main failure report, diagnoses
the actual sink crossbar(s), which can differ from its resource tile and the
recorded congestion tile. It lists each LOCAL terminal's DST, both intermediate
joints, packed-joint reservations, numeric incoming SRCs, and the preceding
DST-to-SRC lease checks. Sources outside the usual docking window are included
and labeled. Full ownership audits of these tiles verify busy nodes against
live design routes and expose orphan leases. This is a read-only **final-state
snapshot**, not a history of attempted decisions: free first hops do not prove
an end-to-end route, and existing forward-anchor lease exemptions are not
applied. It runs only on failure, not in the routing hot path.

Regressions in `fpga.routing`, `fpga.docking`, and `fpga.moving` cover
chronological failure selection, completed/retired-task exclusion, blocked-hop
reconstruction, unchanged leases, and the actual nonzero failed exit producing
a PNG, text report, and stdout tile-state dump.

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

Generic routing finishes one selected trunk before selecting another. Search
branches and their temporary CB states remain speculative until a complete
connection (including Grounding) has been found. A short suffix-depth setting
must not force a task handoff or commit an ungrounded prefix.

Both the normal batch loop and the heartbeat loop invoke `routeTaskOrFail()`.
It requires complete routes from the numeric search, rejects partial commits,
and calls `failRouting()` if the connection cannot finish. The numeric search
keeps a finite forward-search budget (`max_steps=200` per invocation); reaching a
search limit is a failure report, not a proof of physical unroutability and not
permission to park the task for later. Failed speculative branches may backtrack
inside that invocation. Dedicated source/destination movement remains
combinatorial with its own documented limits.

Not-yet-attempted fanouts can wait for their source trunk, and preempted victims
remain explicit repair tasks. Neither case permits the currently running
connection to return unfinished and let the scheduler work on another net.
A stage deadline, stagnation, or task failure terminates routing. Large state
snapshots remain optional diagnostics.

Source deadend masks are learned and enforced only during Basic routing. They
are tile-and-source collision marks: a source is marked after its child search
returns without a committed suffix, whether the failure came from topology or
occupancy. These marks are sticky for the complete Basic stage. Unrouting and
preemption do not clear them. Docking, Moving sources, Fanout routing, and
Moving destinations ignore the persistent Basic masks because they solve
different search problems after occupancy or placement may have changed.

Generic routing needs one completed trunk per physical source port. Once its
seed sink is selected, failure is terminal; replacing that failed sink with a
different deferred sink is forbidden.

### 4. Moving Sources

Moving sources is a trunk-only recovery stage. It handles invalidated trunks,
not failed Basic tasks, and chooses each physical driver as the relocation
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
any source received a definitive repair result. Generic recovery must complete
its current connection or fail; it cannot create partial work for a later
route-first attempt. Selected relocation candidates inspect their own
prefix through a temporary lease-free state view, so reverse probing does not
require queue-wide teardown. Complete trunks, dedicated trees, and deferred
fanout suffixes are not changed.

Generic recovery is change-driven rather than batch-driven. A pass is due
after 256 failed-prefix releases, 32 committed source moves, or 1024 changed
reroute tasks, or when pending preemption victims need to join the workset.
Each recovery pass handles at most 4096 tasks and rotates the
not-yet-attempted workset. Every attempted task must finish before that rotation.
A failed relocation stops the stage instead of advancing to another source.

The reverse route is speculative and owns no live masks while placement is
being selected. `routeOutTry()` uses a non-destructive element-packing preview
and temporary crossbar-state copies to prove simultaneous takeoff for all
connected outputs. The selected trunk output must reach the exact reverse-route
`SRC`; other outputs must each retain a distinct free resolved takeoff.

Input rerouting proofs may extend the input's own retained fabric prefix even
when that route is incomplete. Its source takeoff and landings remain leased;
requiring another takeoff would turn those valid leases into self-congestion.
Sibling anchors must still come from complete routes. A route whose driver is
moving, or another moved input's private path, cannot supply an anchor. The
proof and post-move detachment use the same retained-prefix boundary.

Input-terminal sharing requires a live routed owner. Distributed constant
owners are compared by their declared value, not by their generated source
cell's identity: that cell differs from the logical constant connection.
Opposite values, ordinary signals, orphan leases, and candidate-local
reservations must not be accepted as a matching constant terminal.

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
marker or tile-local endpoint alone is not enough. The existing driver output
is also a valid start: Fanout may add another Takeoff there without removing
the completed trunk.

Fanout branch discovery indexes unique numeric landings from the completed
source tree and visits them in nearest-distance buckets. A preferred fork has
more than two free exits. Usable one- and two-exit forks remain after the
preferred candidates, including when preferred candidates exist but fail.
Source-output starts are tried first. Destination-Tile landings are retained
even with zero free exits, because they may complete directly through Grounding.
The current task searches both classes; it must not yield after a branch slice.
It materializes a shared prefix only after a branch succeeds. Fanout
never re-enters Basic. If Fanout preemption demotes a Generic route, that trunk
is repaired as Generic work inside the Fanouts stage with persistent deadends
disabled; dependent suffixes remain parked until that repair finishes.
Exhausting the task calls `failRouting()`; no partial-branch retry window may
move that failure to another pass.

Fanout search is iterative, multilevel depth-first search inside the current
task. Exhausted children return to their parent to try another outgoing SRC.
The shared source tree stays leased, but unsuccessful private suffixes are not
committed and saved for the next scheduler pass. Top-level Fanout attempts
visit the available branch/target pairs directly rather than using the old
64-pair scheduling slice to postpone the task.

The lower-level continuation and retained-prefix helpers also serve
preempted/imported routes and regression fixtures. Their resumable state is
not authorization for a top-level task to yield an incomplete connection.

Tile repetition is checked over the whole source-to-frontier path, including
the shared branch prefix and every retained private hop, not separately for
each continuation call. At most two visits to one Tile are permitted; adjacent
same-CB transitions do not count as leaving and re-entering it. Each resumed
slice rebuilds its visit seed from the live retained route, so popping a search
level or truncating a preempted prefix also removes those visits. Speculative
alternatives share immutable visit-history roots, rather than one global
visited set. A visit-limit rejection is remembered only for that search path;
it must not become a Tile-wide deadend.

Docking receives that same history plus the current speculative path to its
anchor. Forward growth, backward growth, and the full joined path must respect
the combined limit. Invalid joined paths cannot authorize bridge or terminal
preemption. Exhausted children backtrack normally; valid detours may move away
from the target. This repetition bound is not a maximum length for routes
through distinct Tiles.

Fanout routing considers the connected driver output and follows all completed
routes for the same physical source pin. A downstream branch point is a
destination node already used by that tree where the signal can fork through
an additional outgoing source node or finish at the requested input. For
nonterminal landings, count currently available outgoing fork exits using the
same dynamic lease checks as routing. At least one free exit makes the point
usable; more than two gives it priority, not proof that it can reach the sink.

The branch route copies the shared trunk prefix as shared fragments and owns
only the new private suffix. The already leased trunk destination may therefore
appear in several bindings of the same physical source tree, while each private
outgoing source, joint, destination, and sink local still has one owner. A new
source-Tile Takeoff shares the driver endpoint, not the original trunk's hops.

The follower retains usable lower-capacity points from the trunk and routed
siblings as fallbacks. Preferred points are attempted first, but their mere
presence must not suppress fallbacks. If no trunk or sibling can provide a
successful branch within its search limits, and the source-output attempts
also fail, Fanout routing calls `failRouting()` instead of deferring the task
to Moving.

Branching may reuse the already occupied incoming destination node for the same
net. Leaving that Tile requires a free outgoing source; finishing there requires
only a valid terminal path and available input. Deferred fanouts are
still routed as independent sink tasks; duplicate tasks for the same sink route
are not emitted.

Fanout routing ignores persistent source deadend masks and does not add new
persistent marks. Within one bounded search it still remembers each failed
child edge, returns to that child's parent, and tries another exit. This
task-local rejection is retained on the surviving search levels. Fanout preemption
changes congestion after Generic routing, so an exit learned as unproductive by
an earlier trunk attempt is not a valid permanent exclusion. Real source,
destination, joint, and local leases remain enforced.
If Fanout preemption requeues a Generic trunk repair, that repair remains part
of the Fanout stage policy and therefore does not re-enable deadend masks.

Fanout docking first tries partial bridge victims, then may preempt completed
foreign transit routes. Select victims by their physical dependency on the
conflicting numeric node, not by which binding owns its lease:

- A private branch cut opens a gap and requeues only that binding.
- A shared segment cut opens gaps and requeues every binding traversing that segment,
  including shared replicas and fork arrivals without their own DST lease.
- Siblings that branch before the conflict remain routed. Sharing a source or
  logical net alone is not a reason to remove them.

Validate every dependent binding and the complete replacement path before any
cut. Preview releases for the entire selected group, retaining resources still
used by surviving prefixes, downstream islands or other branches. Detach all
selected transit segments before releasing shared leases. Each
displaced binding is requeued once. Current-source-tree and endpoint owners,
and protected bindings remain protected. If even
one dependent binding cannot be cut safely, reject the candidate unchanged.
A completed foreign transit must not be rejected solely because the current
stage is Fanouts, or because multiple bindings use the contested segment.

Explicit removal of a whole source tree (for example when relocating its
driver) invalidates every dependent binding, not only the binding that exposed
the conflict. This is different from the node-local transit cuts above: those
preserve the takeoff and unrelated branches. Whole-source invalidation is atomic:

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

Moving destinations handles invalidated suffixes, not failed Fanout attempts.
It is still a stage, and its relocation attempts are followed by bounded
Generic and Fanout passes for only the affected task set. The router selects an
unfinished sink endpoint or strict packing cluster and tries positions on tiles
reached by forward routing, using the generic placement legality checks. Ordinary load cells are
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

Destination Moving also seeds forward search from incomplete sibling prefixes
of the same physical driver/output, including siblings stored under different
logical Net objects. A missing sink terminal does not disconnect their retained
fabric. Before exposing these branch points, Moving walks each prefix once:
it must start at the driver's mapped output, have continuous hops and resolved
SRC-to-DST connections, and retain live leases owned by that physical source
tree. A busy DST alone is not sufficient. Siblings affected by the relocation
cannot supply borrowed anchors. No prefix leases are cleared to make this
search possible; a successful suffix shares the retained prefix, and a failed
search leaves the sibling untouched. Basic/Fanouts complete-seed rules remain
unchanged.

The triggering input is forward-routed and grounded at the new position before
the move is accepted. Other input suffixes incident to the moved sink are
invalidated because every old sink local belongs to the previous placement.
Output source trees are invalidated
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

Incident-route search tries alternatives within the current task. There is no
cross-pass retry window for an unsuccessful connection: its failure stops the
stage. Completing one incident route permits trying the next, but cannot excuse
or extend a different failed task.

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

Moving destinations is route-first. A non-directional breadth-first search
grows from the triggering route's retained prefix and the live source tree's
branch DSTs. A Generic seed without a prefix starts at its physical output.
It follows actual SRC-to-DST mappings, not a radial or averaged-coordinate
placement scan. On every reached DST it tests that CB's resource tile and its
attached resource tiles. Disconnected tiles are never placement candidates.

An unfinished private prefix contributes every leased landing DST as a branch
anchor, with its last landing considered first. A blocked tail must not hide
earlier branches. Its old leases remain intact during the search; after a new
suffix commits, the obsolete tail is truncated through ownership-aware cleanup.
The selected landing stays owned by the retained prefix, and any sibling-owned
resources on the old tail remain leased.

An outgoing task of the current focus relocates its actual destination, not
the focused driver or its generated source endpoint. The source and retained
prefix stay in place. After successful relocation the sink becomes the focus;
unfinished work belonging only to the old focus is preserved in the deferred
queue. Incoming tasks may retain their destination cluster's existing focus.
A fixed destination does not authorize source movement through this forward
grounding algorithm; driver relocation belongs to Moving sources.

Packing explores alternative element positions in a reached tile. Acceptance
requires a free terminal from the exact reached DST to the selected input,
including both joints where present. The whole forward prefix is visible as
temporary occupancy during this probe. Other incident endpoints still undergo
local support checks; a terminal without an incoming mapping cannot satisfy
those checks. These checks are filters, not claims of completed other routes.
After reserving the trigger's exact terminal, other affected inputs retain
their constrained-before-flexible reservation order.

The triggering suffix and placement are committed together. A retained private
prefix used by that suffix is preserved, shared donor prefixes remain owned by
their original routes, and the newly grounded trigger is excluded from later
incident-route invalidation. Other affected routes are repaired by the focused
Generic/Fanout passes. A rejected search restores the original placement and
occupancy; no speculative leases survive.

An exhausted Moving destination search immediately calls `failRouting()` to
write the failure report and PNG and exit with failure. It does not retry an
unchanged failed task or move on to unrelated tasks. Candidate rejection may
continue inside the same source search; a false return at its top-level task
boundary is fatal even if unexamined candidates remain.

For a selected `SCALEPNR_CONGESTION_NET`, forward Moving traces include the
actual anchors, expanded states, SRC/DST choices, occupancy rejections and
placement failures. `SCALEPNR_CONGESTION_FORWARD_ONLY=1` limits the trace to
this placement search, avoiding tracing earlier Basic/Fanout attempts. The
ownership audit distinguishes temporary prefix leases from committed leases.

Each forward candidate carries the combinatorial two-visits-per-tile history,
including its retained prefix. A third entry is rejected only for that path;
other branches retain independent histories. Same-CB continuity does not add
a visit. Persistent Deadend marks and directional ordering do not restrict
this search. It uses the enclosing stage's cancellation budget.

Moving destinations also ignores persistent deadend masks and does not create new
persistent marks. It retains failed child edges only within the current bounded
search so the search can return to a parent and select another exit. A moved
endpoint changes the routing problem, so only current physical leases, temporary
per-search failures, and loaded crossbar connectivity constrain its retries.
Every reached destination node inside the grounding radius tries Docking
immediately, before ordinary forward expansion. No deferred position list or
failed-result map is kept. If Docking cannot complete the connection, ordinary
forward search continues from that position and backtracks when its exits are
exhausted.

A failed Generic/Fanout incident task in Moving immediately reports failure.
Backtracking and placement alternatives are internal search operations, not
scheduler slices that permit incomplete routes to survive into later tasks.

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
until the affected Generic and Fanout tasks are both complete. Each incident
task must finish before the next starts. If one fails, report it and terminate;
do not keep its partial growth for a later pass or relocate another focus.

The focused move is atomic. Moving must not restore deferred tasks or select a
different endpoint while any route incident to the current cell or packing
cluster remains incomplete. If its complete placement search is exhausted,
Moving reports that task and exits with routing failure; it does not restart
the same exhausted search. Stage cancellation also terminates the attempt
without exposing partially unrouted incident work as a successful design.

The complete Moving subsequence is:

1. Select one unfinished sink or strict packing cluster as the focus.
2. Grow the triggering suffix forward from a retained prefix or source-tree
   branch, testing packing and grounding at every reached tile.
3. Commit a reached placement only with its complete triggering connection.
4. Detach the other moved input suffixes. If a physical source in the cluster
   moved, invalidate its source tree using the Generic-seed/Fanout-sibling rule.
5. Run Generic routing for every affected source tree that lacks a trunk.
6. Run Fanout routing for every affected secondary sink.
7. Commit only completed connections; continue to another incident task only
   after the current one finishes.
8. If an incident task fails, call `failRouting()` and stop. Candidate search
   and backtracking must occur before this task boundary, not in later passes.
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

For an outcome audit of docking bridge preemptions, set
`SCALEPNR_ROUTE_PREEMPT_AUDIT=1`. The log records each accepted victim cut,
the numeric resources released and their remaining owners, the requesting
search's actual commit outcome, and the whole routing task's bound-route
outcome. This is diagnostic only; it does not change acceptance policy.
Analyze a captured log with
`python3 tests/prjxray/preemption_audit.py LOG --stage Fanouts --json`.
The analyzer checks every recorded cut, reports missing outcomes/coverage,
counts completed victims against actual completions, and checks all recorded
blocker edges for historical route-name cycles. Those cycles are diagnostic,
not proof of a physical resource conflict or a reason to prohibit a cut.
One-for-one completed-route exchanges are reported as neutral, not assumed
beneficial. Multiple binding cuts can share one audit ID: all victims count,
but the common replacement completion counts only once. Takeoff and terminal
preemptions are outside this bridge-specific
audit and are counted separately as uncovered kinds if present.
For the measured failure cases and acceptance defects, see the
[bridge preemption audit](../docs/preemption-audit.md).

A joined Docking bridge is only a geometric proposal. Before cutting its
victim, the router must prove the complete requesting path's leases, including
its terminal, both JOINTs and any preceding generic-search steps. A blocked
terminal is acceptable only if the exact victim cut also releases its occupied
resources. The read-only cut preview excludes the retained prefix and all
resources with surviving owners. Blocked-terminal geometry alone is not a
one-cut proof. Rejected proposals leave the victim untouched. Successful
preemption publishes already-proven lease additions after the cut, with no
fallible lease operation remaining. Only newly acquired bits are published;
restoring a whole saved CB would incorrectly resurrect the victim's removed tail.

Preemption history remains diagnostic only. Physical endpoint protection, not
route-name ancestry, prevents another connection from taking back an endpoint.

Grounding preemption is allowed only for complete terminal paths that connect a
physically incoming destination node to the requested sink local. Before
preempting, the router checks every numeric `DST -> [JOINT...] -> LOCAL` path.
If one path has a free destination and every required joint is also free, no
preemption is allowed. A free destination whose required joint is occupied is
not incorrectly treated as a usable terminal path.

When every usable destination node is leased, the router may displace exactly a
destination node owned by a transit route. A route that uses its destination
node to connect to a local resource in the same tile is an endpoint route, not
a legal victim. The selected transit segment is detached and queued for repair before
the current route retries grounding. Proximity to the destination or docking
radius alone never permits grounding preemption.

Endpoint-owned destination nodes and joints are never grounding victims, even
when an endpoint has another topologically possible entry. Allowing two local
routes to exchange the same joint creates a deterministic preemption cycle.
After a legal transit preemption, the current route retries grounding immediately
and must claim the released terminal path before the victim task is scheduled.
If a physical node is replicated in several bindings of a shared route tree,
all transit owners of that node are removed atomically before this retry.
Moving Sources and ordinary grounding docking use the same preemption operation.
It preflights every owner of every busy DST/joint on a selected terminal path,
including protected routes, completed endpoints, takeoffs, and per-pass/cycle
restrictions. Rejection leaves all routes unchanged and permits selection of
the next eligible entry. One binding owning several joints is cut and requeued
only once; an ineligible shared owner prevents the entire terminal transaction.

A cut requested at a DST must include the hop arriving at that DST. Cutting
only its outgoing continuation would preserve the contested node as the
victim's new `owns_landing` endpoint and would not free the entry. The preceding
uncontested landing remains leased as the victim's retry anchor. A cut that
would remove the source's first physical takeoff, including its landing, is
rejected. Ordinary owner-driven prefix truncation still preserves its landing;
that operation is not a successful preemption of the landing itself. Success
is reported only after every contested lease is clear and every displaced
binding has a pending retry task.

Production-path regressions in `src/tests/fpga/grounding_preemption.cpp` cover
incoming-hop release, protected-takeoff and per-pass rejection followed by a
later candidate, multi-joint owners, shared-owner atomicity, and suppression
of preemption when a complete free entry exists.
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

An occupied bridge may begin directly at the current forward position. Docking
therefore resolves occupied source bits through numeric `dst_by_src` just like
free candidates, but does not traverse them. It reports such an edge only when
its resolved landing is present in the independently reached backward frontier.
This permits exact transit preemption without treating every busy source exit
as a useful bridge.

Bridge preemption detaches each victim at the earliest busy node in that exact
bridge. This preserves its routed prefix and shared source tree while releasing
only the suffix that prevents the frontiers from joining. All bindings that
replicate the same physical bridge are cut together and requeued. The complete
replacement is lease-validated before any cut and committed atomically with it;
Docking does not need to search again after a victim is removed.

Partial bridge victims are tried before completed victims. The number of
dependent bindings is not a single-owner admission limit: cutting a shared
segment can require several suffix repairs. Accounting records every displaced
completed binding, so finishing one requester is not incorrectly reported as
reducing unfinished work when it displaces several routes. Moving-stage,
takeoff, current-source-tree, endpoint, and cycle protections still apply to
every dependent binding.

Grounding may remove either a private Fanout suffix or an entire transit source tree.
When a transit trunk is removed, its Generic seed and dependent Fanout branches are requeued atomically.

Retained-gap repair also obeys free-first Grounding. If its bounded whole-source
fallback finds no free completion, a reached SRC may land on a transit-occupied
DST feeding the reserved sink, directly or through up to two JOINTs. Repair
checks these terminal continuations against its existing reached source paths;
it does not permit intermediate transit to preempt other transit. The router
previews the full dependent cut, validates the complete replacement against
the projected releases and packed-input reservations, then cuts and requeues
every affected binding before installing the replacement. Protected endpoints,
unknown owners, missing terminal arcs and packed reservations reject the
candidate without cutting anything. `fpga.fanout_gap_grounding` reproduces the
recorded three-gap failure and checks this production entry point, free-path
preference, atomic multi-binding cuts and clean resource ownership.

The final jump into the sink Tile must also be considered when its SRC is
transit-occupied and therefore absent from the free search. After the existing
terminal choices, gap repair examines direct outgoing jumps from reached DSTs
that land in the sink Tile. The SRC, landing DST and complete terminal path are
one provisional Grounding candidate: all owners and the projected full path
must pass validation before any cut. This does not authorize preemption along
arbitrary intermediate transit, nor preemption of Takeoff or Grounding owners.
`fpga.fanout_final_jump` captures the missed eight-hop candidate and checks
endpoint protection, free-first selection, atomic dependent cuts and an
unaffected sibling. It reuses reached states without increasing search limits.

Whole-source gap repair recognizes reconnection to the intact terminal suffix
after the binding's last gap. On reaching a node in that suffix, it validates
and appends the remaining physical path instead of rediscovering it through
the fabric search queue. The combined prefix and suffix must satisfy ownership,
incoming-connection and packed-input restrictions, node-cycle prevention, the
two-arrivals-per-Tile rule and the total hop limit. No lease changes occur until
the full path is validated; normal replacement commit preserves sibling owners.
Suffix validation consumes no additional fabric-search states and does not
raise the search budget. `fpga.fanout_retained_suffix` checks this behavior at
the recorded failure boundary, including broken-tail and budget-limit controls.

Whole-source replacement may change the incoming connection of its own private
sink LOCAL while retaining that pin's lease. Privacy requires both exclusive
live ownership by the replaced binding and no references from preserved
bindings, including shared fragments and aliases that do not own a lease.
This exception applies only to the actual sink, never intermediate resources.
Search, final validation and strict reachability diagnostics use the same rule.
The complete replacement is validated before commit; obsolete private Grounding
resources are released only after the replacement is registered. This is not
preemption of another endpoint. `fpga.fanout_private_grounding` checks the
recorded six-hop replacement, old two-JOINT cleanup, shared/alias/foreign guards,
and cancellation, hop-limit, topology and packed-input protections.

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

Each search direction has a hop limit of twice the Docking radius: 10 hops
for ordinary Grounding (radius 5), and 24 for I/O Docking (radius 12).
This bounds graph depth, not geometric distance; the window and beam-width
rules remain unchanged. A joined route can contain hops from both directions.
Explicit smaller/larger depth overrides are available for diagnostic probes.

Each docking attempt owns temporary backward-search memory keyed by crossbar
coordinate and destination-node position. Once a backward position has been
expanded without completing docking, another branch does not expand that same
position again. These temporary docking deadends are discarded when the docking
attempt returns and never modify the persistent crossbar deadend masks.

Incoming transitions are resolved from loaded `dst_by_src` mappings and grouped
by destination `(x,y,dst)`. Grounding first runs the established local free-path
search and returns immediately on success. Only when it fails does Grounding
retry with the complete incoming topology and fresh search counters/visited
sets. Outside alternatives must not consume the local search's breadth slots
and hide a route it could previously find. The wider search still precedes any
preemption; an outside free route wins over a local occupied-terminal proposal.
If both searches fail, local bridge/terminal proposals are retained first,
followed by additional outside bridge proposals. Neither probe changes leases.

The wider Grounding search uses the complete incoming topology:
a wire feeding a reached DST must not disappear because its SRC lies outside
the destination window. The backward search may follow that wire outside and
return through other connections, subject to its existing hop, width, visit,
and occupancy limits. The production router shares the once-built device graph;
these lookups do not rebuild it or copy its incoming lists. Standalone Grounding
without a supplied index builds the complete graph once per call.
An unresolved jump at a chip-edge Tile must not be cached as an empty transition
for every Tile of that CB type; an interior Tile can have a valid landing.

The resolved device topology is shared, but each active search owns its window
center, radius, and memoized incoming lists. Moving Sources can suspend an outer
search to prove a proposed placement's inputs. Those nested searches must not
replace a cache entry still borrowed by the outer search: that used to silently
shrink or enlarge its search region and discard valid continuations.

The backward search uses a bounded breadth-layer beam. Capacity is reserved for
each unexpanded parent in the current layer, not divided permanently among
terminal entries. A terminal entry that has no incoming transition therefore
releases its reservation, while the viable entry can inspect later predecessor
nodes up to the common beam limit. This prevents disconnected terminal entries
from starving a physically routable entry without adding runtime sorting.

`SCALEPNR_AUDIT_DOCK_WINDOW=1` prints `DOCK_WINDOW_RESULT` for each Grounding
attempt: whether local search succeeded, whether wider search was needed, and
the final result. Work counters include both searches when fallback runs;
terminal counts still describe the unique terminal alternatives, not two copies.
This diagnostic does not change search choices or occupancy.

The search is intentionally bounded. The ordinary forward walker stays within
a small square around the destination tile; backward Grounding follows actual
incoming wires across that boundary with limited depth and breadth. Directional
IOB Docking retains its separate corridor/window policy. Docking is still
fully abstract: it uses crossbar masks, dynamic lease state, and device jump
resolution only; it does not inspect vendor wire names.

If an incremental Basic route cannot extend its committed endpoint, the
incoming source is marked as a Basic deadend and only that last committed hop
is removed. This rule is the same after an exhausted docking attempt and after
ordinary continuation failure; the parent crossbar must select another
angle-prioritized exit.

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

### `fpga.target_deadend_commit` - `arena_test.cpp`

A previous terminal-entry failure can leave a persistent SRC deadend even
though a different input on the same destination Tile remains reachable.
When search validates a direct target hop through that mark, its per-edge
exception must also be used when committing the path. This does not erase the
mark globally or bypass SRC/DST/JOINT/LOCAL occupancy checks.

The regression exercises both a one-hop Takeoff and a two-hop transit path
ending through a terminal JOINT. Both must complete despite the old final-hop
deadend. Negative controls occupy the SRC, landing DST, terminal JOINT, or
input pin and require failure without changing existing leases.

### `fpga.immediate_docking` - `arena_test.cpp`

A synthetic bent corridor has a first reached position outside the bounded
completion depth and a second position that can join a complete replacement
through one foreign transit. The route must finish at that second position
without exploring the rest of the corridor first. The old deferred code fails
because only its first fallback position attempts preemption. A negative
control blocks the terminal JOINT and requires the victim to remain intact,
with no unfinished requester suffix published. All fixture names are abstract.

### `fpga.routing_fail_fast` - `arena_test.cpp`

For Basic, source-Moving repair, Fanouts, destination-Moving repair, and Const,
queue an unreachable first connection before a routable ordinary connection.
Verify that the first task invokes the failure-reporting path, the second is
untouched, and speculative SRC/DST/JOINT/LOCAL leases are unchanged. Also verify
that a long routable connection completes in one task despite a one-hop suffix
slice setting. The Moving policy regression rejects retryable-task deferral.
`pnr.clock_fail_fast` exercises both small and batched clock trees with a
blocked second sink: the first stays routed and every later sink stays untouched.
Timeout/watchdog regressions require failure in Basic and Fanouts too, without
the former handoff exemption.

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
- Fanout mode can branch from an existing trunk, a completed sibling, or the
  driver output; a destination DST can ground without a free outgoing SRC;
- `fpga.fanout_source` and `fpga.fanout_destination` reproduce endpoint-Tile
  omissions, check blocked alternatives leave leases unchanged, and remove
  siblings in both orders to verify shared-resource ownership;
- Moving mode releases old cell routes and reroutes the affected hierarchy;
- a route can escape a blocked radius through a one-tile trail initially
  pointing away from its destination;
- bounded continuation commits successful progress and never rolls it back;
- full-net unrouting clears source, transit, joint, local, and tile-pin leases;
- grounding preemption releases the complete victim terminal state;
- remote endpoints require crossbar fabric while attached resource tiles share
  the corresponding route-tile state;
- removing one Fanout suffix preserves its parent destination lease;
- backstepping a private hop transfers its incoming destination lease to the
  retained prefix's landing, including when the parent did not already own it;
- Fanout forks require a destination node already belonging to their trunk;
- equal textual route names retain distinct physical endpoint bindings;
- reattaching one exact endpoint identity does not duplicate or replace a
  completed physical binding;
- moved sinks retain only their reusable source or shared trunk prefix;
- an empty moved-sink binding is still invalidated and scheduled;
- shared terminal Fanouts release only their private local endpoint;
- legacy seed-rotation helper accounting (the stage boundary now forbids
  invoking that fallback after a failed task);
- unfocused Moving repairs cannot preempt, while focused Moving and Fanout
  continuation retain safe grounding preemption;
- generated endpoint chains move in dependency order with their real anchor;
- lease release preserves Basic deadends while clearing every owned node class;
- historical route names do not suppress valid transit cuts or same-pass repairs;
- high-fanout transit cuts retain downstream wiring and unaffected branches;
- shared-gap repair, repeated cuts, cancellation and resource-ownership audits;
- busy transit exits remain visible to preemption candidate selection;
- Basic keeps preempted secondary siblings deferred as Fanout work;
- failed Fanout branches advance their branch-point rotation exactly once.

The suite also runs 64 randomized variants each for direct transit takeoff
preemption, joint-mediated preemption metadata, and preference for a free joint
exit over preempting another route.

Two ownership regressions also run independently as
`fpga.routing_landing_owner` and `fpga.routing_discard_owner`. They use abstract
numeric nodes and cover:

- exact landing-coordinate ownership for shared fragments, including equal
  node numbers in different Tiles and both owner-index lookup paths;
- discarding a branch whose prefix is used by other branches, transferring
  incoming-DST and landing ownership without changing their physical paths;
- same-net and separate-net aliases of one source, successive owner removal,
  private-suffix release, and cleanup of registrations along the shared prefix;
- a clean congestion audit after each removal and no leases after the final
  surviving route is removed.

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

Focused regressions also run independently:

- `fpga.fanout_tile_visits`: a retained suffix has already visited one Tile
  twice, and its next otherwise-free hop would enter a new DST on that Tile.
  Near/far target cases must reject the hop, backtrack through several levels,
  and either find the valid alternative (including an initially outward
  detour) or exhaust the root. Each committed path and all ownership are
  checked; no rejection may leak into Basic's global deadends.

- `fpga.moving_incomplete_sibling`: an empty suffix can reach its only legal
  placement through an incomplete sibling's occupied takeoff. Same-Net and
  cross-Net cases run with and without the source index. Removing either
  branch first preserves the survivor; removing both frees all leases. Invalid
  driver/port, disconnected, ownerless, foreign-owned, missing-lease, and
  co-moved sibling prefixes are rejected. Packing failure preserves placement
  and existing leases. This exercises the actual relocation and commit path.

- `fpga.moving_mapped_io_input`: an I/O resource drives a CrossBar four Tiles
  away through a loaded output mapping, without direct Tile attachment. The
  production input-source probe must recognize that mapped endpoint, reject a
  same-type CrossBar at the wrong offset, and respect both live and trial
  takeoff leases. The former attachment-only check never recognized the source,
  causing exhaustive failed input searches for successive placement slots.
- `fpga.moving_nested_input_window`: keep an outer backward window alive while
  a nested input route uses another window, then resume the outer route. The
  former one-entry window cache overwrote the outer bounds and lost its incoming
  connection. Both searches now own separate views of the same resolved device
  topology; neither proof changes live placement or routing leases.

This suite covers the Moving scheduler and the most recent task-loss fixes:

- moving one Fanout sink releases only its private suffix and preserves sibling
  branches and shared leases;
- pre-move input anchors match the prefix retained by sink detachment; moving
  the driver invalidates those anchors;
- distributed input terminals are shared by constant value only when their
  live route owns the local; opposite values and orphan leases are rejected;
- extending a moved input's private prefix keeps the original owner, frees its
  unused tail, and leaves no leased bits after the replacement route is unrouted;
- forward Moving branches before an unfinished dead tail to reach a legal
  packing tile; success releases the obsolete tail and failure preserves the
  original placement, complete prefix, leases, and ownership;
- backward input proof, bulk sink invalidation, and production suffix commit
  run together for complete and incomplete inputs at two anchor positions;
  invalid retention requests and blocked commits leave existing leases intact;
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
- Docking includes retained-prefix visits when expanding either frontier and
  when joining them. Grounding and I/O cases cover valid bypasses, same-CB
  steps, busy bridges, and blocked terminals. Two individually valid halves
  whose combined path visits a Tile a third time cannot authorize preemption;
- a moved input with no siblings can reuse its own retained private prefix
  without allocating another source takeoff;
- an incomplete owned input prefix remains eligible for reverse docking, with
  unchanged source/landing leases; incomplete siblings and moving drivers are
  excluded by the same eligibility rule used in source-move validation;
- a valid input path beyond 4096 reverse expansions succeeds, while stage
  cancellation still stops search without modifying live leases;
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
- 24 rotated, randomly named anchor fixtures select a three-hop free corridor
  over an eight-hop detour, but retain the detour when the shortcut is occupied;
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

A failure-location fixture retains a committed prefix ending at A and tries
its only free exit into saturated B. It asserts that the task's PNG coordinate
and node identify B, not A; the rejected suffix leaves no fragments or leases.
After freeing B, successful continuation must clear the obsolete diagnostic.

A Fanout fixture has a three-exit fork leading only to saturated tiles and a
one- or two-exit fork reaching the sink directly. Routing must try the latter
after the preferred fork fails, without preemption or changing the shared trunk.
Free-exit count orders candidates; it must not exclude usable lower-capacity
forks. The existing per-task search budget still applies to both classes.

The `fpga.routing` suite also verifies stage progress accounting without wall
clock sleeps: sub-one-percent windows accumulate, a qualifying window resets
the streak, and a single long search accounts for every elapsed stagnant
minute. The default now permits ten deficient one-minute windows (previously
three), independently of the stage deadline. `SCALEPNR_ROUTE_PROGRESS_WINDOW`
and `SCALEPNR_ROUTE_STAGNANT_WINDOWS` override the window and streak limit.
The regression replays the 8,824-task plateau, tests the tenth-window failure,
and verifies that committed progress resets the extended streak.

`moving_source_probes_before_exhausting_reverse_component` in `docking_test.cpp`
uses a neutral 72-tile fabric to verify early placement probing, free takeoff
priority over transit preemption, exact boundary identity, immediate return
after a cut, continued expansion after rejected placements, unchanged live
leases, and cancellation without preemption.

`moving_source_explores_free_continuations_before_preemption` combines an
occupied reverse alternative with a queued free path to a legal placement two
or three hops from the sink. It checks that no preemption callback is invoked,
the accepted path reaches the sink pin, and live masks are unchanged. Bounded
expansion and placement-probe variants must retain unfinished status without
eviction. The exhaustion case above verifies that a necessary cut still occurs
after every free continuation is explored, retaining blockers from earlier layers.
`moving_source_preempts_after_exact_probe_budget_exhausts_free_search` verifies
that rejecting the last reachable placement exactly at the probe budget still
allows the necessary cut; there is no unexamined free work left to protect.

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

Failed Moving searches automatically retain a per-candidate packing trace in
`routing_failure_packing.txt` and print it into the routing log. No selected-net
environment variable is required. The trace records the reached CrossBox/DST,
resource Tile, strict-cluster order, every attempted element position, occupied
slot owners, input/chain/clock conflicts, and backtracking. It distinguishes a
packing rejection from a packed candidate whose grounding or other incident
inputs cannot be supported. Messages are captured during the trial, before its
temporary placement is restored; the failure report identifies the matching net
and whether the packing trace was written.

The automatic trace also snapshots each reached candidate CrossBox (not just
the restored original sink). For rejected other inputs it lists **every**
terminal alternative, including entries without incoming SRC mappings. Per-node
records distinguish committed-mask state, forward-prefix-only leases, and trial
reservations made for the trigger or earlier inputs, naming the reserving route.
`registered_route_owner` is checked against registered route fragments; `<none>`
is not by itself proof of an orphan (use the exhaustive audit below to check
missing registrations). A temporary reservation is not physical congestion.
`OTHER_INPUT_RESERVED` records the greedy choices that led to a later conflict;
these diagnostics do not change the reservation order or add backtracking.

Before a terminal Moving-destination failure exits, an exhausted forward search
is replayed once in diagnostic-only mode. Successful searches do not pay this
rendering cost. The replay uses the same anchors and cannot exceed the original
number of reached states; it does not add earlier branch anchors, commit moves,
or retry indefinitely. A search cancelled by its stage deadline is not replayed.

Each reached CrossBox/resource-Tile candidate gets a numbered text report and
PNG frames in a fresh `routing_failure_candidates_NNN/` directory. Reports include
the committed ownership audit, temporary prefix state, all attempted packing
positions, and terminal reservation conflicts. Candidates with no resources also
get a report and image. Packing rejections are pictured immediately; candidates
that fit but fail grounding/support are pictured while still placed, with trial
input reservations visible, before rollback. Images highlight the reached DST
and reserved input pins. `index.txt` lists every report/image and completion
status; `forward_search.txt` contains the replay's forwarding decisions.
`routing_failure.txt` links the directory, and the packing trace printed on exit
includes the replay reports. Prior candidate directories are not overwritten.

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
registrations and stale registrations. An unfinished crossbar hop must also
have a leased landing: the audit checks that physical requirement even if a
route edit incorrectly cleared its `owns_landing` flag. Indexed net pointers
absent from the provided scope are reported without dereferencing them. Callers must supply
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
The node watch also records lease-bit transitions at those scope boundaries,
with the responsible net and task. A transition automatically enables full
tree history for that net for the remainder of the run, so a later removal of
its route without clearing the lease remains visible. This works without a
net filter when the original owner is unknown. Discovery records explicitly
mark the first unavailable before-tree; subsequent scopes include both trees.
This is a net-tree history, not an instruction-level memory watchpoint; task
boundaries cover direct edits made outside the instrumented mutation routines.
