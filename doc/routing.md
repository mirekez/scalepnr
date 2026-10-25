# Routing Rules

The global execution contract is [run-based PnR](pnr.md). Routing must
finish the current admitted part before the next run; earlier runs' placements
remain fixed. Earlier routes retain their ownership but their eligible transit
segments may be preempted and repaired during the current run; `pnr_preserved`
does not prohibit preemption. Infrastructure and endpoint protections still
apply. Placement/packing should minimize preemptions, not prevent necessary
ones by protecting all earlier-run routes.

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

Classify transit by its node roles, not geometric displacement: a SRC-to-DST
fabric hop may remain inside the same CB. Its incoming DST, SRC, JOINTs, and
explicitly owned landing are eligible transit resources after the protected
Takeoff. Partial-cut planning must handle these same-Tile hops too; terminal
Grounding (without a SRC/landing DST) remains protected. Regression:
[`fpga.fanout_docking_owner`](../src/tests/fpga/fanout_docking_owner.md).

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

When one preemption cuts sibling branches to different downstream boundaries,
each retained tail must own **all** its surviving resources, including borrowed
FORK inputs and later islands. Promoting just the first fragment after the gap
is insufficient: another sibling's longer cut can remove an owner deeper in
that tail. Complete these ownership promotions before releasing any removed
leases. The actual released resources must agree with the preemption preview.
Regressions: `fpga.fanout_cut_island`, `fpga.fanout_cut_island_reverse`.

A retained-gap repair stays runnable in Fanouts even when it belongs to the
original trunk with a private prefix, rather than a shared fanout replica.
It still owns its Takeoff: do not demote it to a new Generic seed or park it
for relocation. Earlier-run fixed endpoints obey the same rule. Deferred
repair work must be consumed or reported as a failure, never ignored when the
active Fanouts queue becomes empty. Regression:
[`fpga.preserved_route_repair`](../src/tests/fpga/preserved_route_repair.md).

A new fanout may share an input still leased by an incomplete sibling. Before
searching into that input, repair its retained owner if it belongs to the same
physical source, then attach through the completed route. Other complete
branches do not remove this dependency. Do not wait for the pending-repair
queue at the end of the pass, clear the input lease, or use a disconnected
downstream island as a completed branch. A failed prerequisite repair calls
`failRouting()` for that owner and reports its gap boundaries. Regression:
[`fpga.fanout_shared_input_repair`](../src/tests/fpga/fanout_shared_input_repair.md).

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
Ownership transfer must continue through physically identical GAPs and common
retained islands: a non-shared GAP record is not the end of the shared route
geometry. In particular, preserve a borrowed FORK input by transferring its
arriving hop's landing before releasing the replaced sibling. Congestion audits
must check both required borrowed-DST leases and incompatible driver/port
claims, not merely that each occupied bit has an owner. Regressions:
`fpga.fanout_island_landing`, `fpga.fanout_island_audit`.
FORK and TRANSIT are ownership roles for the same physical DST-to-SRC hop,
not different inputs. A role difference must not stop shared-prefix ownership
transfer when every coordinate, node and joint agrees. Takeoff still has a
different LOCAL input. Regression: `fpga.fanout_fork_role`.
After free source repair fails, try validated terminal preemption: existing
one-jump completions first, then two-jump approaches whose starting and
intermediate Tiles lie within five Tiles of the sink on both axes. An occupied
first SRC must not hide a legal second jump into the input. Preview the combined
cuts and validate the entire source-to-input path before releasing anything;
Takeoff, Grounding and packing reservations remain protected. This fallback
does not authorize preemption of arbitrary transit outside the Grounding window.
Regression: `fpga.fanout_two_jump_grounding` preserves the recorded interrupted
route and a verified 17-hop replacement using abstract node names.
If these short completions fail, continue numeric search from the reached
states inside that same five-Tile Grounding window, allowing at most ten
additional inter-Tile hops (and respecting the total repair hop/state limits).
Do not restrict this completion to DST seeds or two JOINTs: an eligible
preemptible Takeoff exit and longer intra-CB JOINT chains can be necessary even
when driver and sink use the same CB. Check each obstructed resource's actual
owners, keep all speculative cuts unapplied, then validate the complete path
and combined releases before committing. Completed/protected endpoint owners
and packing reservations remain forbidden. Search-local owner-policy results
are discarded before any subsequent routing task; no persistent occupancy
cache is used. Regression `fpga.fanout_long_grounding` preserves a captured
seven-fragment interrupted route, the contested 15-fragment transit owner,
and its verified three-hop/four-JOINT replacement with abstract node names.
After 64 exact-gap search states, try a free-only whole-source repair using
the caller's normal state budget. Do not truncate that early probe to a much
smaller budget and then flood the exact-gap region before repeating the same
source search at full size. Failed speculation changes no leases; exact-gap
search and the preemption-assisted fallback remain available. Cancellation,
hop limits and atomic completion requirements are unchanged.
`fpga.fanout_gap_probe_budget` checks this ordering with a deterministic work
budget and an abstract, deliberately broad search frontier.

Keep negative ownership checks cheap during repair. For a busy exact-gap SRC,
first find an actually owned, same-source arc with the required incoming DST;
only then validate every physical owner. Whole-source repair can reject busy
nodes absent from its already-built source-node sets before an ownership scan.
These are negative filters, not permission to share: candidate shared arcs and
all provisional preemption cuts still receive full owner validation. No persistent
occupancy cache is introduced. `fpga.fanout_gap_owner_scan` and
`fpga.fanout_gap_owner_source` check the expensive-query counts, valid shared
reuse, unchanged foreign routes and clean node ownership.
This fallback uses the repair state/hop limits and cancellation checks; the
optional whole-device reachability audit is diagnostic, not the routing loop.
Intermediate LOCAL wires have ordinary routing leases, not endpoint pin
leases, and remain eligible for transit cuts and subsequent repair.
For a new Basic trunk or fanout with no saved route, exhaustion of the ordinary
hop/branch search also permits this bounded numeric source search, trying free
paths first. Basic and Fanouts must use the same completion helper: supporting
internal LOCAL bridges only for fanouts leaves valid first trunks unroutable.
The fallback follows actual numeric arcs and occupancy rather than treating a
deadend learned by the narrower hop topology as proof of disconnection. It must support
internal LOCAL connections away from Grounding, preserve exact same-source
inputs and packed reservations, and install only a complete validated path.
Do not create a placeholder live binding or reparent another binding's
detached tail to perform this search. Existing successful branch searches
remain the fast path. Regression: `fpga.fanout_transit_local` and its direct
and disconnected controls.
After all original numeric attempts fail, a new task may make one additional
bounded free-prefix/Grounding attempt with half the geometric bias. An original
successful route and its selected victims must remain unchanged: changing its
priority can increase preemptions and perturb later incremental placement.
The free fast path, per-attempt state/hop limits, stage deadlines and preemption
rules stay unchanged. The alternate attempt does not repeat the unchanged
Takeoff probe. A cheap outward detour can otherwise miss a legal Grounding
entry while nearer congested corridors consume the frontier.
If all normal 32,768-state attempts for every placed pin pair still fail,
the task retries those pairs with a 65,536-state numeric frontier, followed
only on failure by a final 131,072-state frontier. Every earlier budget
visits all pin pairs first. This is bounded recovery, not an unrestricted
device flood. Original successes return
before recovery, so their routes and victims stay unchanged. Each low-level
call respects its explicit state bound; hop limits, cancellation, packed-input
checks and atomic complete-path preemption remain in force.
`fpga.basic_grounding_detour_recovery` includes the alternate-priority arcs
missing from the first reduced capture. Both original 32K priorities fail;
the task-level recovery must complete the route, retain and repair its victim,
and pass real ownership audits.
`fpga.basic_free_detour` reproduces a later run-6 failure where both 64K
priorities miss a fully free 21-hop continuation through internal LOCALs.
The 128K caller recovery must finish it without preemption or moving cells;
separate low-level-limit and disconnected controls forbid silent budget
expansion, invented connectivity, and leaked leases.
`fpga.basic_grounding_detour` imports the validated 17-hop run-6 approach and
its private transit obstruction; budget, protected-owner and disconnected
controls check that only a complete eligible replacement permits a cut and
that the displaced route can be repaired.
`fpga.source_fallback_precedence` checks that an original one-victim success
is retained even when weaker geometry would choose a two-victim approach.
`fpga.basic_transit_local` imports the free ten-hop run-6 witness, including two
internal LOCAL bridges. Its direct and disconnected controls isolate topology
support and check that a failed search commits no speculative route.
Bounded numeric completion must not spend frontier states on an unrelated
LOCAL with no outgoing physical arc. Exclude only a proven topological leaf,
not all input pins or LOCAL nodes: internal LOCAL bridges and dedicated
cross-Tile LOCAL links remain legal, and the required sink is always eligible.
This is independent of occupancy/deadend heuristics. Regression
`fpga.basic_search_budget` imports a recorded free frontier and a 24-hop
witness; the state and hop limits are unchanged.
Cross-Tile LOCAL templates must resolve at the actual Tile: a nonempty
template list whose required neighbour types are absent is not a physical
continuation. When no intra-CB successor exists, reject that leaf before
allocating a search state. Regression: `fpga.basic_local_templates`, with
recorded free arcs, unresolved templates, and budget/disconnection controls.
The same structural proof may follow a short JOINT/LOCAL chain before
allocating its intermediate states. Prune only a chain proved to have no
useful exit; reaching the bounded lookahead limit means unknown and must
preserve the candidate. Source/sink nodes, fabric exits and usable dedicated
LOCAL networks remain eligible. Regression: `fpga.basic_remote_local`.
Before searching a retained GAP, check the live owners of its final LOCAL.
If a completed same-source sibling already reaches that exact physical input
from the same physical output, reuse its complete route through the normal
shared-terminal attachment. Publish it before releasing the old private
prefix. Do not treat a gapped sibling, a different LOCAL, or a foreign owner
as a reusable completion. A known completed path must not be lost behind
the numeric search budget. Regression: `fpga.fanout_complete_input`.
Rank a numeric SRC using its physical landing, not only its departure Tile.
Otherwise a useful long jump can remain queued behind short exits until the
state budget is exhausted. Retain SRC-to-JOINT alternatives, and keep all
normal occupancy, input-continuity and complete-path checks when expanding
the selected exit. This is search ordering, not permission to bypass blocked
resources. Regression: `fpga.fanout_source_priority`; its documentation
distinguishes the reduced fixture budget from the real-chip failure.
Numeric fanout completion must also prefer the connected tree: first restrict
the initial departure to an existing same-source SRC, then permit a new
Takeoff only if that bounded attempt fails. A shorter independent source path
must not bypass this preference. Regression: `fpga.fanout_transit_local_reuse`.
A completed seed may consist entirely of numeric `WIRE_ROUTE_EDGE` fragments,
including a local path through several JOINTs. Do not classify it as a missing
trunk because it has zero legacy CROSSBAR fragments. Recognize its source
LOCAL and numeric DST arrivals in the ordinary Fanouts branch index; retain
the same completeness, ownership and endpoint checks. Regression:
`fpga.fanout_numeric_seed` and its full-flow, legacy, disconnected and numeric
arrival variants.
If no free completion exists, a reached numeric prefix must still receive the
normal bounded Grounding search through eligible transit obstructions. Use the
same preview, whole-path validation, atomic cut and victim scheduling as gap
repair; a new task must not silently lose that capability merely because it
has no retained route. Existing-source departure remains preferred before a
new Takeoff, and protected Takeoff/Grounding owners remain ineligible. No cuts
are permitted for a disconnected or cancelled candidate. Regression:
`fpga.fanout_numeric_grounding`, its protected-owner and disconnected controls,
including repair of the preserved victim and lease-clean teardown.
Every preemption callback, including ordinary Takeoff, must classify a victim
with a retained gap as a repair of its existing source tree. Do not defer that
repair to Moving as a new Generic trunk while its siblings wait in Fanouts.
Keep the complete-trunk barrier and repair the preserved path first.
Regression: `fpga.fanout_takeoff_handoff` and its explicit-repair control.
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

## Shared Crossbar Data

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
deficient windows latch `stagnation_detected=1` and enable verbose per-net
diagnostics to stdout. `SCALEPNR_ROUTE_STAGNATION_ATTEMPTS` sets the number of
additional full work cycles (default 3; 0 requests immediate failure). A cycle
is one outer per-net routing/recovery invocation, including its nested docking,
placement probes and immediate input rerouting. Nested calls do not consume
`stagnation_attempts`. The remainder of an invocation already running at
detection is logged but not counted as a full cycle. Each subsequent full return
decrements the counter; at zero the standard routing failure handler prints
the state/ownership audit and renders the failed-net PNG. Every stage, including
Basic, uses that terminal diagnostic failure policy. A failed task or hard
deadline still fails immediately, even during a diagnostic cycle; Basic must
not hand failed work to Moving or advance to another task. A qualifying window
before detection
resets the deficient-window streak. Search and relocation cancellation points
poll the same watchdog, so one long speculative operation cannot hide a stalled
stage until its hard deadline. The window and streak are configurable through
`SCALEPNR_ROUTE_PROGRESS_WINDOW` and `SCALEPNR_ROUTE_STAGNANT_WINDOWS`.
The trace identifies the net, operation, source/sink, retry and branch offsets,
before/after committed fragments, backward frontier pops and parent indexes,
accepted and rejected edges, busy-node live/search masks and ownership,
placement candidates, takeoff/input-proof rejections and final search counters.
Logging is observational: it adds no routing filters, sorting or leases.
`stagnation_diagnostics_count_only_full_outer_attempts` checks partial and nested
attempts, exact countdown, repeated watchdog polling, reset and zero allowance;
`stagnation_runs_full_diagnostic_attempts_before_cancelling` exercises production
watchdog/recovery entry points, stdout, successful completion, stage reset and
hard deadlines. `backward_search_trace_records_free_busy_and_probe_results`
checks that free and busy decisions are both printed without altering the
chosen route or leases.
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
keeps a finite forward-search budget (`max_steps=200` per starting exit for
whole-connection searches, with a 200-hop depth limit). An exhausted starting
exit must yield to the remaining exits of the same task, not fail the entire
task immediately. Its speculative arena is reused; budget exhaustion must not
publish a physical deadend or commit the abandoned suffix. Stage cancellation
still applies. Failure after these bounded attempts is not proof of physical
unroutability and not permission to park the task for later. Failed speculative
branches may backtrack inside that invocation. Dedicated source/destination movement remains
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

Forward placement search reconstructs paths one depth layer at a time. Only
the current layer invokes placement probes, once per candidate and in the same
breadth-first order across all anchors. Earlier layers rebuild the active
prefix without probing it again. RAII restores temporary leases at each return;
diagnostic observers receive the same original, pre-prefix Tile states. Visit
counts are derived from the active path and retained prefix. Storage therefore
grows with path depth rather than the number of alternative paths. There is no
new depth, expansion or memory cutoff, no global visited set, and no change to
the existing two-visits-per-tile rule or numeric source ordering.
A third entry is rejected only for that path;
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
[bridge preemption audit](preemption-audit.md).

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
sink LOCAL or a private JOINT in the sink Tile while retaining the pin's lease.
An alternative DST may therefore reconnect a disconnected Grounding without
being forced through its obsolete incoming DST. Privacy requires both exclusive
live ownership by the replaced binding and no references from preserved
bindings, including shared fragments and aliases that do not own a lease.
Foreign owners and packed-input reservations remain protected.
The complete replacement is validated before commit; obsolete private Grounding
resources are released only after the replacement is registered. This is not
preemption of another endpoint. `fpga.fanout_private_grounding` checks the
recorded six-hop replacement, old two-JOINT cleanup, shared/alias updates and foreign guards,
and cancellation, hop-limit, topology and packed-input protections. It also
checks replacing the incoming arc of a retained two-JOINT tail, including
non-owning sibling and alias replicas. `fpga.fanout_private_grounding_repair`
checks real incremental preemption followed by repair through a different DST
and attachment of a dependent shared-input sibling.

An additional safe rejoin applies to disconnected final tails of interrupted
bindings with the same driver and physical sink LOCAL. A different incoming
connection may reach their retained SRC, JOINT or LOCAL only when no completed
binding, live prefix, or other destination's retained tail references that
resource. All interrupted consumers of that physical input are replaced
together, retaining each binding's own endpoint metadata. Source-prefix
ownership is transferred only to bindings outside the replacement group; every
replacement is registered before old leases are released. There is no
preemption or mutation during the search. Search limits are unchanged.
`fpga.fanout_retained_src_rejoin` imports the recorded source-to-sink bypass;
`fpga.fanout_shared_terminal_gap` checks the shared terminal-JOINT variant.
The protected-destination control rejects without changing any binding.
The read-only `preserve_inputs` audit intentionally preserves existing shared
arcs; its relaxed mode can expose candidates requiring coordinated replacement.

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

The append-aware numeric net-index regression verifies that each generated
passthrough adds exactly one indexing visit, unchanged lookups add none, vector
relocation keeps lookups valid, and duplicate aliases and transaction resets
retain the original lookup semantics.

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
- `fpga.moving_forward_memory`: reject parking throughout a generic 20x20
  mesh until 1,000,000 production placement probes have run. The search must fit
  within 48 MiB of additional address space and restore every temporary lease
  on cancellation. The former full-Node frontier and persistent visit tree fail
  this regression with `std::bad_alloc`, as does a merely compacted retained
  frontier. No new search limit or pruning rule is introduced by the path-only
  fix. A separate multi-anchor fixture verifies exact breadth-first probe order
  and independent leases for different paths to the same Tile.

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
Forty additional randomized cases verify the entire priority sequence, not just
its first node, with fully loaded, empty, and partially populated priority masks.

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
It also measures logical versus uniquely allocated mask payloads and verifies
that common bitmask and annotation containers remain shared across subtypes.

### `fpga.cb_sharing` - `cb_sharing_test.cpp`

Independently constructed synthetic crossbars with randomized insertion order
share equal tables while retaining different jump destinations. Tests cover
hash collisions, all mask words, copy-on-write mutation and clearing, weak-pool
lifetime, derived-table rebuilding, and read-only routing queries that must
not detach storage or insert missing rows. Subtype annotation and dedicated
local-transition overrides must leave their base and siblings unchanged.

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

A production takeoff-preemption fixture registers a local departure, a transit
using the same SRC number on another tile, and the actual contested SRC owner.
It verifies that only the actual owner's suffix is removed, its source prefix
survives, the new local takes the freed exit, and unrelated leases remain intact.

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

Set `SCALEPNR_ROUTE_PACKING_TRACE=0` to disable the full temporary packing
trace spool on memory- or disk-constrained hosts. The default remains enabled.
This only removes that detailed packing artifact: routing decisions, ordinary
stdout statistics, per-net congestion tracing, terminal snapshots and failure
visualization are unchanged. In particular, a tmpfs-backed temporary directory
must not be assumed to have room for every speculative placement probe.

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

For an independent free-path check, set `SCALEPNR_AUDIT_SOURCE_REACHABILITY`
to the canonical RTL net name (substring match) and
`SCALEPNR_SOURCE_REACHABILITY_LOG` to a fresh filename. Besides interrupted
route repair, `failRouting()` runs this opt-in audit for an ordinary failed
task with no stored route. It starts at each real placed output LOCAL and
checks each placed input LOCAL without registering diagnostic bindings or
changing leases. It respects foreign owners, existing same-source inputs and
packed-input reservations, but ignores routing deadend heuristics. A reported
path is a reachability witness, not a committed route; `limited=1` means the
diagnostic state bound prevented an exhaustive answer. It never preempts.
For a failed unbound task, `SCALEPNR_SOURCE_REACHABILITY_STATES` can raise or
lower this diagnostic-only bound (default 1,000,000; valid range 1–8,000,000).
Invalid values retain the default. This does not change any routing limit or
permit committing diagnostic paths. A limited result must not be reported as
proof of disconnection.

For an unknown next failing net, set `SCALEPNR_AUDIT_FAILED_SOURCE=1` instead
of guessing a name before placement. At `failRouting()`, the actual failed
task records each source/sink resource coordinate, CB coordinate and LOCAL
ID (`FAILED_SOURCE_PAIR`), followed by live source/sink ownership audits.
The same snapshot is replayed read-only at up to 131,072 states with the two normal
priorities, recording numeric arcs and parent states. If neither finds a
complete witness, a larger preview-only probe runs without per-edge logging.
`SCALEPNR_SOURCE_PREEMPTION_AUDIT_STATES` controls that larger bound (default
4,000,000; 1–8,000,000). `SCALEPNR_SOURCE_REACHABILITY_STATES` still controls
the separate free-component audit. Both write to
`SCALEPNR_SOURCE_REACHABILITY_LOG` or the failure directory's default log.
No normal routing attempt gets a larger budget, no victim is cut, and the
original failure still propagates even if a witness is found. A bounded
miss is not proof of disconnection. `auditRouteCompletion()` selects the stored
binding when one exists, retaining its gaps, shared prefixes and private
terminal ownership. Its endpoint arguments must match that binding. The
free-component audit likewise uses `auditNetRouteReachability()` for retained
routes. Treating such a task as unbound used to silently skip the completion
probe and lose the private-terminal exemption in the reachability probe.
This avoids confusing snapshots when timed placement changes which connection
fails first between diagnostic replays.

`fpga.fanout_retained_frontier` replays the free numeric frontier captured from
the 51,749-cell run on 2026-10-24, with synthetic node and cell names. The real
task stopped in chunk 15 with a retained gap from (58,121)/DST3538 to
(54,115)/DST3538. Its source was (82,162)/LOCAL142 and its sink was
(58,89)/LOCAL84. All 199 leases at the failure tile had real owners; the
source and sink audits were also clean. The corrected completion probe found
a 22-hop path, without preemption, after generating 35,158 states, exceeding
the then-production 32,768-state search bound. Retained-route repair now uses
`DEFAULT_ROUTE_REPAIR_STATES = 262144` in both its API default and the router.
The 200-hop limit, cancellation checks and ownership validation are unchanged;
that retained-route fix did not alter the separate unbound-task budget sequence.

Removing unrelated chip topology reduces the replay's completion to 32,527
states. The positive-control test uses the production default and checks
ownership and teardown. `fpga.fanout_retained_frontier_capacity` exercises the
lower-budget limitation at 30,000 states: a failed attempt must
leave the retained binding and leases unchanged, while a larger read-only
probe must produce a completion witness without mutation. A future repair
that fits the smaller budget is also accepted and audited. Increasing the
production budget does not prove all larger frontiers will fit. An experiment
ranking toward the retained tail instead of the final sink did not resolve
this case and was reverted; the budget increase does not change routing priority.

The 65,536-state retry completed chunk 15 and 14,987 cells, then failed in
chunk 16 at (54,80)/DST245. Its 170 leases, plus the source's 26 and sink's
171, had no ownership inconsistencies. The retained connection was
(45,197)/LOCAL146 -> (57,51)/LOCAL97. A read-only probe found a 64-node free
completion after 155,801 states, without preemption. Both 131,072-state probes
failed. `fpga.fanout_retained_wide` imports the eligible numeric frontier and
retained 69-fragment binding with synthetic names; it reproduces the failure
at 65,536 states. This evidence motivated the current 262,144-state bound.
The fixture verifies complete routing and lease teardown at the production
default, rather than treating a diagnostic witness as a committed route.
Its text snapshot groups each source `(x,y,kind,node)` with a target count
and that many target tuples, avoiding repeated source fields on every edge.

The 262,144-state retry completed 17 chunks and 16,986 cells, including both
previous failures, then stopped on an unbound Basic task in chunk 18. Its
endpoints were (41,184)/LOCAL144 and (53,98)/LOCAL91. Independent audits found
all 171 leases at the failure tile (82,142), 28 at the source and 199 at the
sink correctly owned and registered. The 131,072-state completion attempts
exhausted their budgets; a larger read-only probe reached the sink through
DST3810 without preemption after generating 490,171 states. This was neither
a timeout nor proof that the sink was disconnected.

`fpga.unbound_completion_frontier` replays the two captured free frontiers
and successful witness using generated names and numeric topology. The old
32K/64K/128K production sequence fails in this isolated test. Failed or cancelled
attempts must leave all leases and bindings unchanged. A successful completion
must create one complete binding at the original endpoints, preserve the
independent owner of a busy destination alternative, and release only its own
resources when unrouted. `UNBOUND_ROUTE_STATE_BUDGETS` is shared by the router
and test: 32K, 64K, 128K, then 512K states. Every earlier budget still visits
all endpoint pairs before the new last-resort attempt. Priorities, preemption
policy and the 200-hop bound are unchanged. This remains a finite limit, not
a guarantee that every routable task fits within it.
The reduced replay completes a 29-hop route in 150,377 states at the final
budget; its first three attempts reproduce the old capacity failures.

`src/tests/fpga/extract_route_frontier.py` extracts numeric replay groups from
the failure log. It resolves each search's private node indexes separately
before merging arcs and adding the completion witness. Device, resource,
cell and net names are not copied into the fixture. The replay contains the
eligible free subgraph, not a claim to reproduce all chip occupancy; live
occupancy is checked independently in the captured failure audits.

The next native retry (`sparse_20261024_232232`, seed 5000, chain length 10,
51,749 original cells) passed 17 of 52 chunks, completing 16,986 cells, then
hit chunk 18's 1,200-second Basic deadline. Total elapsed time was 2:00:11;
peak RSS was 1,823,148 KiB. All 207 CTests passed before this retry. The last
minute window reported 257 -> 248 remaining Basic tasks; this is not the
final task count. This run produced no completed design database.

The interrupted task connected (71,145)/LOCAL141 to (75,98)/LOCAL97; the
reported search frontier was (49,143)/DST546. Independent ownership audits
checked 166, 140 and 215 leased resources on the failure, source and sink
tiles respectively, with no unowned leases, missing leases/registrations,
stale registrations or conflicting drivers. A larger read-only probe found
a free completion after generating 713,435 states, with no preemption.
This proves a free path at the captured state, not that the interrupted
normal task had exhausted every production attempt. The failure was a
stage deadline, not a proven disconnected endpoint or ownership leak.

The isolated completion regression's profiled search attempts took about
132, 285, 619 and 680 ms. Sampling identified numeric graph lookups and
unconditional node-name materialization as costs, but does not attribute
the native stage timeout to a specific defect. Its reduced free subgraph
does not reproduce the complete chunk's ownership, preemption or aggregate
workload. No further routing change is justified by this profile alone;
the next optimization needs a reproducible workload and measured benefit.

The following retry (`sparse_20261025_080720`) again stopped at chunk 18's
Basic deadline: 17 chunks / 16,986 cells completed, last minute-window count
193 (not a final task count), elapsed 1:54:32, peak RSS 1,830,116 KiB. All
436 sampled leases on the failure/source/sink tiles had valid owners. The
interrupted task had a free completion in a 97,119-state read-only probe.
The heartbeat accounted for about 1,172 seconds inside routing tasks but
only 40 seconds in the hop fast path; numeric completion was the expensive
fallback, not a proven ownership leak.

`fpga.source_dominance_work` reproduces redundant work in that numeric
search with 48 neutral tiles and 1, 8 and 32 reconvergent lanes. The old
ordering walked parent chains and checked eligibility before discarding
labels already dominated in both cost and hops. The 8-lane case generated
753 states but made 3,328 eligibility checks and 159,706 ancestor visits.
The existing dominance rejection now precedes that work; this does not
introduce a new path restriction. Its best-label table uses a numeric hash
lookup because no operation depends on key order. Queue priority, insertion
order, hop/state limits and preemption policy are unchanged.

The regression bounds operation counts rather than wall time, verifies
complete numeric connectivity, checks cancellation/unroute ownership, and
preserves an independently routed busy arrival. Full frontier diagnostics
still validate and record dominated eligible edges for replay; the test
requires the same selected route with diagnostics enabled and disabled.
On the captured unbound frontier, free-search CPU time across its four
budgets fell from 1.58 to 1.22 seconds (about 23%), with unchanged state
counts and completion depth. This measurement is isolated search time,
not a claim about the full 50K run's speed or completion.

After this change, all 208 CTests passed (849.36 seconds). Native retry
`sparse_20261025_112926` passed chunk 18 Basic, 1,029 -> 0, with no trunks
left for Moving sources. Chunk 16/17 routing took 371.9/398.8 seconds versus
549.4/693.5 in the preceding run; placement variation also affects these
native timings. Fanouts then failed on an unbound connection, not a deadline
or stagnation stop. The run completed 17 chunks / 16,986 cells, elapsed
1:56:45, peak RSS 1,832,692 KiB. No final design database was produced.

The new request is (66,48)/LOCAL161 -> (67,117)/LOCAL0, with reported
frontier (11,131)/DST3538. All 358 sampled leases were correctly owned.
Normal completion failed through its 512K bound. A read-only 1M-state probe
also exhausted free search, but its Grounding continuation found a 39-hop
path requiring SRC193 at (63,119) and DST193 at (63,117). Both cuts belong
to one real transit fragment of a retained partial route; they are not
orphan busy bits. This is a preview, not a committed repair.

The captured two 131K free frontiers plus the witness contain 147,165 source
groups / 592,963 arcs, not the full failed 512K or larger preempting search.
The free-frontier extractor does not recreate victim ownership; feeding its
two-cut witness into a free-only fixture would wrongly free those nodes.
This next capacity/preemption case has not yet been faithfully reproduced
as a regression. Further budget or search-policy changes remain unverified.

For a complete replay capture, enable `SCALEPNR_CAPTURE_COMPLETION_FRONTIER=1`
alongside `SCALEPNR_AUDIT_FAILED_SOURCE=1`. The read-only failure audit then
records the production-size unbound frontier and the larger diagnostic probe's
eligible arcs and indexed states. It does not enlarge production search budgets.
The extractor refuses a witness requiring cuts unless `--cuts-output` is given;
that numeric manifest identifies occupied nodes whose real owner routes must be
reconstructed in the regression. A cut witness must never become a free-only
fixture. All diagnostic output can remain in the single compressed run log.

### Numeric Terminal Fanout Reuse

The next instrumented run, `sparse_capture_20261025`, again finished chunk
18 Basic (1,029 -> 0). Fanouts started with 6,057 tasks, then spent over ten
minutes on one connection while its pending count stayed at 5,814. The
stagnation diagnostic recorded 7,650,849 events for that connection. It
eventually completed, as did three diagnostic cycles; the latched watchdog
then cancelled the next task. Thus that last cancelled task was not evidence
of an unreachable endpoint: its read-only probe found a free 60-node path
after 496,962 states. The run stopped after 17 chunks / 16,986 cells, elapsed
2:09:14, peak RSS 1,786,828 KiB.

The slow connection's destination crossbar was (57,59), LOCAL64. JOINT56 was
reported busy 4,890 times for each of several terminal alternatives. The
ownership proof identifies a completed sibling from the same physical driver:
its numeric tail uses DST194 -> JOINT56 -> JOINT41 toward the sibling's pin.
The new sink can branch at JOINT56 directly to LOCAL64. However,
`attachSharedDestinationLocalFanout()` recognized only legacy CROSSBAR
terminal fragments, not the equivalent ROUTE_EDGE terminal chain. Fabric
search consequently treated this reusable sibling joint as an obstacle.

The fast path now scans the existing numeric terminal chain for a direct
numeric connection to a free requested local. It verifies the branch node is
leased by the same physical source and the retained prefix contains no GAP.
Only that prefix is borrowed; the old sink's remaining tail is not copied.
One new ROUTE_EDGE and the new pin are appended, and only the new local is
leased. No fabric-search priority or preemption policy changes are involved.

`fpga.fanout_numeric_terminal` reproduces the omission with neutral names and
one-, two-, and four-joint chains. It failed before the fix. It requires zero
fabric-search edge trials, unchanged sibling leases, the exact new numeric
terminal edge, complete ownership registration, and independent suffix
unrouting that frees the new pin but leaves the sibling intact.

`fpga.resource_pin_audit` covers a separate diagnostic defect exposed by this
run: resource-side pin leases were reported as unowned because the audit and
its spatial index inspected only route-CB coordinates. Endpoint metadata now
supplies those resource-side claims. Such annotations do not require an extra
resource-side lease or CB registration for ordinary generic routing. The test
checks both indexed and full-design audits, a genuinely leaked neighboring
pin, a removed endpoint association, and annotation without a second lease.

### Post-Fix Deadline Replay

All 210 regressions passed after the numeric-terminal and resource-pin-audit
fixes (729.04 s). The next native run, `sparse_20261025_173205`, completed
17 chunks / 16,986 cells, then hit the 1,200 s Basic deadline in chunk 18.
Elapsed time was 1:52:04 and peak RSS 1,748,176 KiB. Its last progress window
reported 192 -> 185 pending trunks. This chunk never reached Fanouts; the run
does not demonstrate that the preceding Fanouts stall is solved in the native
design, despite the focused regression passing.

The cancelled task was `slice$109632/Q -> parse_blif$154932/I2`, memory bit
`modules_42.Queue4_UInt32_2.ram_ext.Memory[2][27]`. Its source crossbar is
(82,115)/LOCAL146, destination crossbar (71,48)/LOCAL81, and resource tiles
(83,115) and (72,48). The saved search marker is (110,136)/DST261, not a
committed prefix. The marker/source/sink audits respectively verified
147/131/158 leased nodes with zero missing leases, unowned leases, conflicting
drivers, or registration errors. The backward neighborhood report additionally
audited 2,010 leases across 13 tiles without those inconsistencies.

The read-only probe found a free completion in 38,224 states, without cuts.
Its coordinate witness is below; this is a diagnostic path, NOT the cancelled
task's committed routing. Each `SRC -> DST` pair is one numeric physical jump:

```text
(82,115) LOCAL146
(82,115) SRC1027 -> (92,115) DST1029
(92,115) SRC1026 -> (97,115) DST1025
(97,115) SRC1027 -> (106,115) DST1029
(106,115) SRC977 -> (111,111) DST977
(111,111) SRC1027 -> (122,111) DST1029
(122,111) SRC977 -> (126,107) DST977
(126,107) SRC161 -> (126,100) DST161
(126,100) SRC161 -> (126,94) DST161
(126,94) SRC161 -> (126,88) DST161
(126,88) SRC161 -> (126,82) DST161
(126,82) SRC3537 -> (122,77) DST3537
(122,77) SRC3537 -> (116,73) DST3537
(116,73) SRC3073 -> (106,73) DST3078
(106,73) SRC3074 -> (102,73) DST3072
(102,73) SRC161 -> (102,67) DST161
(102,67) SRC3537 -> (97,63) DST3537
(97,63) SRC3073 -> (86,63) DST3078
(86,63) SRC161 -> (86,57) DST161
(86,57) SRC161 -> (86,50) DST161
(86,50) SRC3809 -> (85,49) DST3809
(85,49) SRC3073 -> (75,49) DST3078
(75,49) SRC3809 -> (71,48) DST3809 -> LOCAL81
```

Chunk 18's phase timers attributed 778.692 s to free search (3,354 calls),
233.530 s to takeoff completion, 106.577 s to short terminal completion,
25.928 s to full terminal completion, and 2.838 s to commits. These are
aggregate phase costs across many tasks and pin pairs, not time spent on the
last cancelled task alone. The stage deadline is real; an unreachable last
sink or a phantom busy mask is not demonstrated by this capture.

`fpga.unbound_deadline_frontier` replays the 81,225 captured eligible arcs
(31,335 source groups), using only numeric geometry and generated neutral
names. It finishes in 32,741 states under the existing 32,768-state limit,
and checks cancellation, complete endpoint attachment, foreign ownership,
and cleanup. This positive control does not reproduce the whole-stage
deadline: omitted, unexpanded alternatives and earlier tasks can affect the
native search workload. No additional search-policy change is justified by
this replay alone. Full native routing remains unverified.
The final full regression run, including this positive control, passed all
211 tests in 739.49 s (`/tmp/scalepnr-deadline-all-tests.log`).

`fpga.fanout_retained_audit` covers retained-route completion diagnostics with neutral names:
a retained prefix and disconnected terminal have a valid alternate entry.
It checks a positive completion witness, an endpoint-mismatch rejection, a
genuinely disconnected negative case, unchanged leases/bindings during every
probe, and agreement with an actual production repair.

`fpga.basic_detour_failure_capture` checks capture without a preselected net,
the actual endpoint coordinates, numeric frontier and a known free witness,
while requiring the original error and unchanged routing ownership. The
Grounding detour regression also checks a positive preview requiring transit
preemption: the diagnostic must not cut the victim or attach a requester.

The independent report also records `SOURCE_CLOSEST` and per-Tile
`SOURCE_REGION` prefix witnesses inside the five-Tile Grounding window.
These refer to the whole audited free component, not just the bounded
router's frontier. For a failed unbound task, additionally setting
`SCALEPNR_SOURCE_PREEMPTION_AUDIT_STATES` (1–8,000,000) runs an opt-in larger
probe using the actual terminal-preemption callback. It previews complete
paths and combined cuts, but stops before any cut, lease or route attachment.
`SOURCE_DIAGNOSTIC_COMPLETE` proves a preview-valid path, not a committed
repair; the original task still fails. This probe omits per-edge frontier
logging unless `SCALEPNR_CAPTURE_COMPLETION_FRONTIER=1`, to avoid
multi-million-line logs by default. It does not increase normal routing
budgets. A limited probe does not prove no legal preempting path exists.
Before the larger probe, it tries weaker priorities with cost scales 4 and 8
using at most the smaller of the normal and requested state bounds. A found
witness records its scale and bound, and skips further probes. Every such
trial remains read-only, and the original failed task is never rescued.

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

### Dedicated-Chain Input Capacity

Packing a producer also obliges its dedicated local consumer and that
consumer's other producers to fit in the same Tile. Checking each prospective
member independently against the current occupants can overbook the crossbar's
fabric-input budget: both producers are accepted, but the consumer's selector
then exceeds the limit despite having a free element lane.

Future-consumer admission counts distinct external drivers over its dedicated
input ancestry together with existing crossbar owners. Shared signals count
once, and dedicated internal connections and global distribution do not consume
ordinary arrivals. This is a read-only capacity check; exact lane, endpoint,
and joint checks still apply. No routing leases or speculative placements are
created by this check.

`fpga.packing` includes `future_chain_input_demand`: real resident input
ownership, both producer insertion orders, an over-budget rejection before
anchoring, exact-capacity acceptance, shared-input deduplication, preview
cleanup, and agreement between preview and actual packing. Instance, Tile,
and crossbar names are synthetic.

#### 5,000-Cell Chunk Retry, 2026-10-25

Seed 5000, chain limit 10, original 51,749-cell design. The focused regression
failed before the capacity fix and passed afterward. The full Release build
and all 211 CTests passed (743.12 seconds). The native retry retained the
1,200-second per-stage deadline, 6 GiB virtual-memory cap, and 200 MiB host
available-memory guard.

The original chunk-2 failure was a three-member dedicated chain needing seven
external signals where only six remained available. The capacity check now
rejects that location before anchoring a producer. The entire chain subsequently
packed at resource Tile `(10,7)`, with the consumer at position 137.

| Chunk | New Cells | Completed Cells | Placement Seconds | Routing Seconds | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| 1 | 4,998 | 4,998 | 23.455 | 5.880 | Complete |
| 2 | 4,999 | 9,997 | 331.579 | 20.425 | Complete |
| 3 | 5,000 | 14,997 | 316.902 | 98.818 | Complete |
| 4 | 5,000 | 14,997 | 324.065 | Fanout stage deadline | Failed |

Chunk 4 completed 4,369 clock sinks, 4,366 constant tasks, and all 5,160 Basic
trunks. Fanout work repeatedly displaced existing routes: completed passes
reported `14498 -> 14197 -> 14337 -> 14584`. The last progress-window count
was 13,638, not a final completed-pass count. The process exited with code 1
after 41:49.20 wall time, peaking at 1,785,528 KiB RSS; no memory guard fired.

The deadline interrupted the branch ending at register `slice$104874/D`,
driven by logic cell `parse_blif$164346/O`. Resource endpoints were `(33,14)`
and `(52,79)`, attached to crossbars `(32,14)` and `(53,79)`. The saved search
frontier was `(15,44)/DST817`, not a committed route endpoint. The destination
was local 23 through joint 15. Two terminal DSTs (23 and 544) were free;
this does not prove a complete free path from the source.

Authoritative source, sink and failure-tile ownership audits were clean.
The backward terminal report audited 13 Tiles, all with zero unowned leases,
missing leases, missing/stale registrations, and conflicting drivers.
The diagnostic production search without the expired deadline found a
60-node completion from source local 155 (70 nodes from alternative local
163). Both witnesses require releasing SRC3841 at `(54,79)` and DST3846 at
`(53,79)`. These are real leases on one shared transit segment: 12 branches
reference it, with one owning claim. They are not leaked busy bits.

Before failure, profiled recovery phases included 650.187 seconds of free
source search, 185.228 seconds of takeoff completion, 96.266 seconds of short
completion, and 42.577 seconds of exact-gap search. These are cumulative
phase measurements, not a separate end-to-end timing measurement. The
live-state completion probe succeeds; no further isolated routing-correctness
failure has been reproduced from this stop. The preceding preemption/task
sequence remains the unresolved investigation, so no search-policy or
preemption-policy change was made for it.

Artifacts are under
`tests/prjxray/nextpnr_runs/size_132_seed_5000_20260822_112830_118638/chain_10/candidate_001_seed_5000/sparse_20261025_204710/`:
`run.log.gz`, `routing_failure.txt`, `routing_failure_backward.txt`, and
`routing_failure.png`. This retry did not produce a completed design database.
