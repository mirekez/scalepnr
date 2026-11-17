!!! Never put in these documents any results of tests or their lists/descriptions or results of experiments or unsure things or general talks. These files only for short requirements / explanations !!!
!!! This is the main requirements file, not history or protocol of changes, be laconic, write worthy things, feel free to reasonably shorten this file or statements here !!!

# Placing

The global execution contract is [run-based PnR](pnr.md): Estimate ranks
the entire design, then each admitted part completes placement and routing
before the next part starts. Completed parts remain fixed.

Whole-design **Estimate** precedes the run-local placement stages: **Outline**,
**Placing**, **Sorting**, and **Swapping**. Estimate belongs to the global PnR
orchestrator; its placement-related principles are described below. Estimate
and Outline work with logical groups and continuous coordinates. Placing
commits cells to exact device resources. Sorting opens space by shifting short
row or column cascades, and Swapping performs bounded timing repair on the
resulting legal packed design.

## Top-Level Placing Requirements

The rules in this section are the primary placing contract. They describe the
intended behavior of the placer; implementation details later in this document
must preserve them.

### 1. Estimate

Estimate must classify the design into independent, timing-aware trees of
cells. A tree is divided into **bunches** of work. Each bunch starts at a
register or another sequential boundary and owns the combinational chains that
feed it. Links to preceding registers or boundaries connect bunches into a
forest.

The classification must retain timing criticality. Path delay, combinational
depth, total path length, and timing deficit must be propagated through the
forest so that the most timing-constrained bunches and links can be processed
first.

Consecutive register-to-register links with no combinational logic between
them are zero-length chains. Compatible zero-length chains must be merged into
the same bunch where doing so preserves clock ownership, tree links, and cell
ownership. They must not be treated as independent groups that consume outline
space without need.

### 2. Outline

Outline is the preliminary spatial stage. It must make the structure of a
dense netlist visible before exact resource assignment. Three useful names for
this principle are **unraveling the tangle**, **untangling knots**, and the
established graph-layout term **force-directed graph layout**. The classic
literature also calls the attraction analogy **force-directed placement** or a
**spring-embedder model**; Graphviz uses the shorter term **spring model**.
These names describe the principle, not a requirement to reproduce one
particular published algorithm. See
[Fruchterman and Reingold's force-directed placement paper](https://doi.org/10.1002/spe.4380211102)
and the [Graphviz spring-model description](https://graphviz.org/docs/layouts/neato/).

The physical analogy is to take a knotted collection of ropes, stretch it over
a larger area until spaces and junctions become visible, and then let related
points pull toward one another. Equivalently, the cell groups are a mass of
stars: Outline first distributes them around a two-dimensional universe in a
cyclic pattern, then applies attraction along their connections.

Attraction must be timing-aware. A link with a worse timing deficit must exert
more influence than a relaxed link, while fixed cells and package constraints
remain immovable. Attraction must be balanced by spreading pressure: groups
that make an area too dense must move as groups toward regions with spare
capacity. Outline must therefore leave both connected groups near one another
and enough room for legal exact placement.

Outline produces continuous, architecture-independent coordinates. These are
preferences, not commitments to Tiles or element positions.
Outline does not legalize these preferences against exact per-Tile element
capacity. Several cells may deliberately retain the same preferred Tile so the
exact placer can smear them with net and timing context instead of destroying
their locality early.

### 3. Placing

Placing assigns every placeable cell to an exact Tile and an exact element
position. It must smear each outlined group over nearby two-dimensional space
rather than collapse the whole group onto one Tile. If a group is too dense for
its preferred area, the placer must shift the group, or a legality-preserving
part of it, toward nearby capacity.

The central legality operation is an accurate estimate of whether a Tile or
site can accept a cell. It must consider more than the nominal count of free
elements. The abstract `Element` model is the authoritative description of
packing capacity: it describes the element positions in a Tile and the local
cell-chain relationships between them. At minimum the acceptance operation
must account for:

- the cell's abstract element type and the Tile types that provide it;
- occupied and free positions within the Tile;
- dedicated local connections, ordered chains, and lane compatibility;
- connected cells that must share a Tile or a particular relative position;
- cell combinations that share, reserve, or block another element; and
- local endpoint or routing capacity when that check is enabled for the
  placing mode.

The stage completes only when every placeable cell has a legal assignment.
Failure to place a cell after bounded recovery passes is an explicit placement
failure, not a successful partial result.

### 4. Sorting

After exact packing and timing refinement, Sorting visits setup endpoints below `-0.100 ns` in worst-slack order and independently corrects their launch and capture cells, skipping fixed cells.
Rebuild the list after each pass; stop when no move is accepted or a time/pass limit is reached, without resetting the direction cursor.

Rotate the preferred evacuation direction N -> E -> S -> W once per movable cell's shift attempt, whether it succeeds or fails; fallback directions do not reset this order.
Try only directions that let the selected cell approach its destination while displaced occupants move in the opposite direction.

Pack displaced Tile contents from the vacancy back toward the target, then place the selected cell in the freed position without overshooting its destination.
If the first compatible axis fails, try the other; never evacuate occupants in the selected cell's movement direction.

Derive movement distance from half the endpoint's negative slack; accept only legal `Element` packing with predicted all-input timing improvement for that endpoint.
A worsened neighbor must remain better than the endpoint's original slack; TNS is not an acceptance constraint, and committed shifts are never rolled back.

### 5. Swapping

Swapping is the final repair stage after smearing, exact packing, and
placement-aware timing refinement. It must recalculate placed timing, visit
violated endpoints in criticality order, and inspect the most expensive
driver-to-sink edges on each critical path.

For a separated pair `A` and `B`, the larger coordinate difference selects the
working axis. A horizontal separation searches a row strip and a vertical
separation searches a column strip. The default strip width is five Tiles.
Within a bounded distance along that axis, the stage searches for another
movable bunch `C` whose location would put the bunch containing `A` or `B`
closer to its critical peer. Fixed bunches and fixed I/O anchors are never
challengers and are never moved.

Each proposal is a transactional two-step relocation, not a direct exchange.
Restoration uses the same relaxed routing-capacity policy as initial placement
and trial relocation. Element, lane, clock, and concrete input ownership still
apply; restoring a saved position must not newly impose strict routing-joint
capacity. Both Swapping and timing-refinement regressions reproduce a failed
restoration using two distinct input locals sharing a mandatory joint and verify
recovery with the unchanged placement policy, without displacing the other owner.
First, the critical bunch claims `C`'s anchor Tile. Second, `C` receives an
independent bounded search around the timing-weighted centroid of its external
connections; it is not sent to the critical bunch's old Tile. Registers and
associated combinational cells are then packed around the selected anchors by
a small timing-driven Manhattan search. Every new position must pass the normal
abstract `Element` and Tile legality checks. `C`'s calibrated external wire
delay may degrade by no more than five percent. The proposal is committed only
if the selected endpoint's timing deficit improves by at least five percent,
total negative slack does not increase, and worst slack does not regress.
Otherwise every affected cell, Tile position, outline coordinate, and bunch
coordinate is restored exactly before the next challenger is tested.

A strongly corrective exchange has a bounded exception: when the selected
endpoint deficit improves by at least 80%, both global TNS and global WNS
deficit may regress by at most 5%. This permits a large local repair to cross a
small temporary global tradeoff while preventing an exchange from creating a
materially worse critical path.

Swapping repeats this traversal for a configurable number of passes. Timing
and criticality ordering are rebuilt after every accepted exchange, and each
pass receives a fresh bounded attempt budget. A pass that accepts no exchange
terminates the stage early because another identical traversal would have no
new placement state to inspect.

## Main Principles

The data becomes progressively more concrete:

```text
RTL connectivity and timing
        -> forest of timing-aware bunches
        -> continuous 2D bunch and cell outline
        -> exact Tile and element position
        -> timing-driven row/column sorting
        -> bounded timing-driven bunch exchanges
```

The separation is intentional. Estimate decides which logic belongs together;
Outline decides roughly where groups should live; Placing decides whether an
exact device resource is legal. An earlier stage must not pretend to know the
full answer of a later stage, but it must provide enough information to keep
the later search bounded.

The placer must also preserve these principles:

- Fixed I/O and user-constrained cells anchor the solution and must not move.
- Timing criticality controls work order and attraction strength; connectivity
  alone is not sufficient.
- Density is measured against compatible capacity, not against a uniform count
  of Tiles.
- The model remains architecture-independent. Device data describes abstract
  element types, positions, and connections; placing algorithms do not depend
  on third-party architecture names.

## Current Implementation

The current flow is connected in
[`Tech.cpp`](../src/tech/Tech.cpp). `Tech::openDesign()` runs Estimate, while
`Tech::placeDesign()` fixes assigned I/O cells, runs Outline, runs exact
Placing with placement-aware timing refinement, runs Sorting, and finally runs
Swapping.

### Estimate implementation

Estimate is implemented by
[`EstimateDesign`](../src/pnr/EstimateDesign.cpp), with its persistent
tree data in [`RegBunch`](../src/pnr/RegBunch.h).

`EstimateDesign::findTopOutputs()` scans top-level output connections, follows
the configured buffer-port mapping, and creates the root `data_outs` bunches.
`estimateDesign()` then recursively builds each tree, aggregates empty register
chains, and sorts the result.

`recurseReg()` owns a sequential boundary for one `RegBunch`. It identifies the
clock connection using the technology's clocked-port table, associates a known
clock and period when available, skips the clock input, and walks every other
input backward. A preceding register or buffer creates a `BunchLink` and a
child bunch. A preceding combinational cell enters `recurseComb()`.

`recurseComb()` continues the backward traversal through combinational logic.
For each arc it obtains a delay from the technology combinational-delay table
and accumulates bottom and top path statistics. Each visited combinational cell
is assigned through `bunch_ref`, so it has one primary owner. If another path
reaches an already-owned cell with a larger bottom delay, the traversal may
grab the logic for the more demanding path. When traversal reaches another
sequential boundary it creates a link containing:

- accumulated delay;
- timing deficit, normally accumulated delay minus the bunch clock period;
- combinational-chain length; and
- a `secondary` flag when the upstream bunch already belongs to another tree.

Both register and combinational recursion propagate maximum total length,
maximum combinational depth, maximum delay, and maximum deficit through the
instance statistics. Bunches separately count their own and inherited register
and combinational cells.

`aggregateRegs()` merges a structurally simple run of zero-combinational
bunches when parent and child use the same clock, both have one child, and the
parent has one uplink. It repairs every `bunch_ref` and child `parent` pointer
after moving the child lists. The current traversal limits the empty-chain
aggregation depth with a guard of four.

`sortBunches()` orders root bunches, child bunches, and uplinks by descending
maximum deficit, with delay as the tie breaker. This makes Estimate's output
timing-prioritized before Outline starts.

### Outline implementation

[`OutlineDesign`](../src/pnr/outline/OutlineDesign.cpp) uses a fixed 10-by-10 logical mesh, independent of device dimensions.
[`OutlineGrid`](../src/pnr/outline/OutlineGrid.h) provides coordinate and iteration helpers.

#### Fixed I/O anchors

`placeIOBs()` and its hierarchy fallback map package pins to Tiles and site positions, then fix the cells and their bunches at the corresponding mesh boundary.
Match sites by relative coordinates, falling back to site order; resolve both input and output connections before trying instance names.
Index pin and package Tile names once so the work grows linearly with the number of I/O cells.

#### Bunch distribution and attraction

`optimizeOutline()` allows roughly one bunch pass per ten reachable cells, with a minimum of one and a maximum of 100.
It calculates average LUT capacity per coarse mesh box and uses a finer occupancy grid with twice the device width and height.

`recurseRadialAllocation()` spreads branches between fixed I/O anchors, using their centroid and graph depth to position descendants proportionally from their parents.
Fixed bunches stay in place; branches without another fixed descendant use a cyclic perimeter walk: down, right, up, left.

`prepareSharedCombLinks()` adds missing links between consumers and the bunches owning their shared combinational logic, without duplicating existing uplinks.
These links pull their endpoints directly; they do not change cell ownership or the Estimate tree, or pull entire register subtrees.
Fixed bunches remain fixed, and clock connections do not create data attraction.

Each pass pulls linked bunches together when they are more than one mesh step apart, with stronger attraction at 75% and 95% of the clock period.
Secondary links can move connected groups recursively; primary links move only their endpoints to avoid repeated whole-tree work. Fixed bunches never move.
Movement steps decrease from `0.1` to `0.05`, then to `0.01` during density refinement.

The legacy density phase after pass 100 is inactive under the current 100-pass cap.
It rebuilds box occupancy, moves bunches from overloaded boxes toward a least-loaded box in the searched half of the mesh, and draws large combinational bunches toward the center.

#### Cell smearing

Calculate all timing-attraction movements from unchanged coordinates, then apply them together.
Each clocked cell combines vectors toward its connected cells, weighted by distance and timing pressure.

For critical register-to-register paths, subtract intrinsic delay and minimum local-wire costs from the clock/setup allowance to obtain a wire budget.
Add attraction only while the endpoints' calibrated Manhattan span exceeds that budget; average competing corrections and stop each correction as soon as the span fits.
Registers drive these movements; fixed/completed cells stay still, and no timing-tree traversal is needed per movement step.

Paths with no positive wire budget keep ordinary timing-weighted attraction.
Do not impose a zero-distance target when intrinsic delay alone prevents the path from meeting timing.

Fixed I/Os pull connected combinational cells toward their anchors, fading by half per hop for at most four steps; registers generate their own attraction and fixed boundaries stop propagation.
Apply calculated movements without an occupancy veto, and let register windows follow their updated positions.
Combinational followers obey chip bounds and step limits, not a small owner-bunch window, so they can span distant anchors.

Exact capacity legalization is deferred to `PlaceDesign`. Outline's final
coordinates remain the centers from which physical radial smearing starts,
including when more cells prefer one Tile than its `Element` model can accept.

### Placing implementation

Exact placement is implemented by
[`PlaceDesign`](../src/pnr/place/PlaceDesign.cpp). The abstract element and
Tile acceptance model is implemented by
[`Element`](../src/fpga/Element.h) and [`Tile`](../src/fpga/Tile.cpp).
`Element.h` is the architecture-independent packing model used to describe a
Tile's available cell positions and the chains that may occupy them; it is not
only a primitive-type enumeration.

#### Candidate preparation and spatial search

Before exact placement, `PlaceTiming::preparePlacementGuide()` traverses the
clocked timing cones and assigns additional pressure to their data nets.
Tighter required periods produce stronger pressure. If a cone's estimated
setup time already exceeds its requirement, its pressure is additionally
multiplied by the normalized timing deficit. Ordinary non-clock nets retain a
baseline physical wire cost, preserving locality for paths that are not
currently part of a clocked cone.

`preSmearBunches()` reserves legal element positions for whole bunches before
committing them. It preserves their Outline shape where possible and uses
compact multi-Tile envelopes for oversized bunches. Reservation search visits
complete Manhattan rings around the requested position. Movable exact shapes
with crossbar input metadata first compare routing-pressure cost within four
steps, then ring distance and timing-weighted wire cost to external peers;
measured routing pressure takes precedence when available. Beyond that
neighborhood, or without input metadata, once home and the immediate ring
cannot fit the search compares two extra rings. Timing cost uses the weighted
cube of calibrated external wire delay, so several short connections do not
cheaply outvote a long one. Fixed groups stay at their origin, and oversized
groups first grow a compact legal envelope. Right/down alternation remains a
tie-break, not a restriction on available directions. Each candidate must pass
the exact `ElementPackingPreview` checks; arithmetic capacity alone is not
enough to stop the search. Resource accounting follows the positions actually
reserved, including compact envelopes.

Guide crowded-area placement using an equal blend of the last successful bunch displacement and the average; consult adjacent Tiles only when the original Tile has no history.
Ignore hints shorter than two Manhattan Tiles; otherwise shift the search origin and projected positions of unplaced peers, preserving bunch shape and fixed cells.
Update hints only after successful reservations, keep normal legality checks, and discard the temporary data after pre-smearing.

Pack cells in a stable order with dedicated same-Tile predecessors before consumers, choosing the first legal free element without rearranging earlier choices.
Reuse prepared member lists and filter candidates using actual free-element counts, refreshed after accepted bunches and including paired-column reservations.
If a member fails, undo only the current bunch's trial reservations and try another location; preserve earlier bunches.

Reserve physical input ownership for constants before their distributed routes are created: equal constants may share an input, but different signals may not.
Enforce this even when external routing-capacity checks are disabled; local constant distribution does not consume external-input capacity.

Single-cell bunches may try up to 16 shared-input Tiles: prefer the home Tile without pressure data, or zero-extra-input reuse before nearby pressure-ranked alternatives when data exists.
Keep small fanouts local, allow farther reuse for larger shared controls, and retain radial placement for multi-cell groups; all normal packing checks still apply.
Update the preview index only on success, clear it when previews end, and rebuild it from committed placements before ordinary placement.

Relaxed initial packing resolves each input's physical endpoint directly,
without first constructing an unused union of all input masks. Endpoint
ownership is checked before walking neighboring chains. Element-owner lookup
scans the Tile's existing instance references, tests the numeric position
first, and does not allocate a temporary instance list. No new ownership cache
is introduced by these shortcuts.

Input ownership checks visit non-clock pins before the Element's declared
clock pin. Shared-control conflicts can therefore reject a candidate without
repeating a compatible clock lookup. The clock is still checked when the other
inputs pass; its name and group come from the Element model, not a hardcoded
primitive convention. Regressions use a `TICK` clock pin to check both early
control rejection and rejection of a conflicting deferred clock.

Connected constant inputs participate in the same physical endpoint ownership
checks as ordinary signals. Their negative designators do not mean that the
input is unused: the connected global driver identifies the constant before
Const routing creates its route tasks. Equal constants may share an input;
different constants or a constant and a signal may not. A negative designator
without a connected global constant driver does not reserve an endpoint.
`constant_controls_reserve_shared_input_endpoints` in `packing_test.cpp` checks
both insertion orders, equal and unequal drivers, cached and uncached lookup,
preview rollback, committed placement, and unconnected inputs.

`SCALEPNR_PROFILE_PLACEMENT=1` adds `PLACE_PRE_SMEAR_PROFILE` to the existing
minute-by-minute progress output: origin scans, capacity candidates, exact
packing calls, successful bunch reservations, and time spent in each operation.
`PLACE_PIN_LOOKUP_CACHE` reports immutable mapping lookups, not occupied-node
state. The progress elapsed time starts at reservation search; database loading
and Outline are outside it. An outer process timeout therefore is not a
Detailed-placement time budget.

A fixed I/O anchor stays fixed, but its movable combinational followers may
reserve nearby space without moving that anchor or its bunch center. An
actually fixed member cannot be translated. A failed shape reservation never
falls back to translating the shape to `(0,0)`; it reports failure if no legal
location exists. This avoids converting a local capacity conflict into a
chip-wide timing detour.

`smearOversubscribedCells()` converts the Outline into a frozen physical-Tile
snapshot and identifies crowded Tile/element-type groups on every cooling
pass. Those groups seed a bounded connection search through four register
tiers. Every reached clocked cell originates exactly one personal movement;
combinational cells never originate gravity. For each affected register,
`calculatePredictedDirections()` adds the distance- and timing-deficit-weighted
vectors to its connected cells and records the resulting force and
acceleration without changing any position. LUT-like and dedicated
combinational cells do not receive independent gravity.

`applyPredictedDisplacementSimultaneously()` implements one simultaneous
continuous movement within each cooling pass. A register displacement in
physical Tile units is its saved force multiplied by a calibrated scale no
larger than `SCALEPNR_PLACE_REDISTRIBUTION_STEP`, `0.20` by default. The scale
is reduced when necessary so the fastest register moves at most two Tiles;
the final accumulated displacement of every follower is capped to the same
two-Tile distance. The displacement is
propagated through its connected combinational constellation with a factor of
one half per hop. Propagation stops before another register because that
register already has its own saved force from the same frozen picture. Fixed
boundaries and the connection-depth limit stop further traversal. Contributions
from all registers are accumulated from the frozen snapshot, then every
continuous Outline position is published together. As in Outline, movement is
bounded to the physical window around the owning coarse bunch, preventing
repeated local cooling steps from becoming an unbounded walk to an edge. The next pass then takes a
new frozen snapshot and recalculates all forces. Thus a collapsed group can
acquire a connected shape without traversal order allowing an earlier cell to
misorder a later one. There are no preassigned destination Tiles, capacity
reservations, integer minimum steps, or fallback directions. By default the
cooling process runs `ceil(device_width)` passes. The CMake cache setting
`-DSCALEPNR_PLACE_REDISTRIBUTION_PASSES_FACTOR=<factor>` scales that duration,
and `-DSCALEPNR_PLACE_REDISTRIBUTION_STEP=<scale>` changes the scale ceiling.
`placement_motion` retains the force and applied continuous displacement for
diagnostics. `PLACE_SMEAR_PASS` reports each recalculation and
`PLACE_SMEAR_SUMMARY` reports cumulative movement and calibration ranges.

During exact packing, `tryAddTimingAware()` converts each provisional
coordinate to a physical preferred Tile and searches complete Manhattan rings
around it, bounded to four steps and 32 exact packing trials. Candidates are
ranked by additional routing-pressure cost, then distance, then calibrated
horizontal, vertical, and bend delay to every connected cell. Missing crossbar
input metadata gives zero pressure cost and retains nearest-ring priority.
Already placed peers contribute their exact Tile coordinates; unplaced peers
contribute their Outline coordinates. A net leaving an oversubscribed group
therefore pulls its cell toward the corresponding side of the group instead of
letting Tile-array order or a rotating cursor select an arbitrary direction.
Occupancy is only a tie-breaker after timing cost, and every commitment still
passes `Tile::tryAdd()`.

`recursivePackBunch()` follows the Estimate forest and the input connectivity
inside each bunch. The continuous outline coordinate is scaled to a physical
Tile coordinate before the search. Cells already fixed or placed keep their
Tile and have their drawing coordinates normalized to the committed position.

Dedicated chains override the ordinary search order:

- a carry-chain predecessor is packed first, and its successor prefers the
  vertically adjacent coordinate required by the chain;
- strict local-chain input drivers and their sibling drivers are packed before
  the consuming mux-like element;
- a strict local-chain sink is deferred until all required drivers are placed;
  and
- an already placed strict peer anchors the candidate to its one legal Tile,
  so scanning other Tiles cannot create a false solution.

Unanchored cells use timing-directed physical radial smearing. Anchored cells
use the dedicated-chain search, with a strict same-Tile anchor restricted to
one trial. Search coverage is sized to cover the device rather than using an
arbitrary small radius. If no legal candidate exists, the current code prints
the cell and terminates placement.

After the forest walk, `placeDesign()` makes up to 64 cleanup sweeps over every
packable cell in the hierarchy. Each sweep retries unplaced logic and prints
`PLACE_SWEEP` counts. It stops when all such cells are placed or when a sweep
makes no progress. Long searches report `PLACE_PROGRESS` once per minute, and
the stage writes `place_output.png` for inspection.

The placement puzzle disables debug rendering by default so its five-minute
limit measures placement rather than PNG generation. Set
`SCALEPNR_PLACING_PUZZLE_PNG=1` to restore the movement frames and final debug
image when visual diagnosis is required.

#### Exact Tile acceptance

`elementTypeForInst()` maps each generic placeable primitive to an abstract
element class. The current classes distinguish two LUT roles, two mux levels,
carry logic, and registers. A `TileType` describes up to 16 positions per class
with `Element::bitmap_pos`. Each element also carries left/right blocker masks
and its column relationship, allowing the model to represent which positions
form a local cell chain. A concrete Tile mirrors that model in `elements_pos`,
`elements_free`, `elements_left`, and `elements_right` bitmaps. Availability is
therefore recomputed from the loaded element topology and the cells already
packed into that Tile, rather than from a fixed cells-per-Tile number.

Blocker masks express mandatory packing dependencies, not all selectable local
connections. In a site with an independent fabric data input for a register,
the site-to-Element builder does not make that register a blocker of the local
mux chain. Thus a fabric-fed register cannot reject unrelated LUTs through
unused mux lanes. Sites without that input retain their mandatory chain links;
occupied strict mux chains, input-node ownership, and paired-LUT restrictions
are still checked. The normal packing regression covers both input topologies,
preview/commit agreement, and packing all eight LUTs before or after registers.

#### Routing-pressure preference

Pre-smearing exact movable groups and the ordinary bounded per-cell search
therefore compare legal alternatives within four grid steps. They prefer the
smallest increase in squared input pressure, then distance, timing cost, and
the existing deterministic tie-breakers. Pressure is the number of distinct
external driver connections divided by the loaded crossbar input-local count.
This is a congestion estimate, not a proof that those inputs can all be routed.
Resource Tiles sharing a crossbar share this demand. Equal node/cell names do
not identify signals; driver connection identity does. Repeated inputs from
one driver count once. Internal connections, global distribution, and dedicated
clock inputs do not count as ordinary ingress.

The preference adds no capacity limit and makes no changes to Element blocker
masks, endpoint ownership checks, or routing leases. Every selected candidate
still has to pass exact packing. Fixed groups cannot move; atomic local groups
keep their shape. Beyond the bounded neighborhood, the existing nearest-ring
search remains available. Oversized groups keep their envelope search. With
no input-node model the preference is neutral. Live preview ownership is read
directly, so rolled-back reservations cannot leave cached pressure behind.

`placing.routing_pressure` runs twelve reproducible seeds for each of seven
scenarios: independent mixed banks, shared-driver banks, connected LUT/register
groups, a full single-Tile device, fixed groups, ordinary per-cell independent
placement, and ordinary per-cell shared-signal placement. Capacities vary up to
eight LUTs and sixteen registers; origins include edges/corners and member order
is shuffled. Checks cover concentration, complete placement, chain co-location,
unchanged fixed coordinates, dense legal fallback, and fanout reuse. Additional
focused checks cover same-name/different-driver identity, heterogeneous input
capacity, shared-crossbar resource Tiles, clock/global exclusion, and exact
preview rollback. Run together with `fpga.packing`, `placing.timing`, and the
placing puzzle tests: reducing density by breaking legal packing is not a fix.

#### Exact acceptance implementation

`Tile::hasFreeElement()` is only the fast capacity filter. `Tile::tryAdd()` is
the committing acceptance operation. It calls `tryElementPlacement()`, which
enumerates free bits and accepts the first position for which both `canHost()`
and `neighborsCompatible()` succeed.

`canHost()` verifies that the Tile models logic elements and enforces slot-level
co-location between connected LUT and carry logic. `neighborsCompatible()` is
the more complete feasibility test. It checks occupied blockers in both
directions through the element graph, the direction of real netlist
connections, exact lanes of strict local chains, convergence of sibling
drivers onto one future sink lane, and space for not-yet-placed chain members.
It also prevents illegal paired-LUT overlays and accounts for full-width LUTs
that reserve both paired roles.

The same acceptance machinery can check conflicts between local input routing
endpoints and joint reservations. Initial `PlaceDesign` calls
`Tile::tryAdd(inst, false)`, so that routing-capacity portion is currently
disabled during initial exact placement; element occupancy and chain legality
remain enforced. Modes that pass `true` receive the additional endpoint
capacity check.

On acceptance, `Tile::tryAdd()` updates the type counters, records the encoded
position and Tile on the instance, clears the chosen element bit and any paired
reservation, invalidates affected placement caches, and refreshes Tile-local
state. Rejected candidates make none of those changes.

#### Placement-aware timing refinement

After exact packing, [`PlaceTiming`](../src/pnr/place/PlaceTiming.cpp) traverses
the timing forest built by `Timings` from clocked data inputs. It retains the
same shared combinational subpaths, but adds a wire estimate to every placed
driver-to-sink edge. The default architecture-independent calibration is, in
nanoseconds:

```text
0.010 + 0.035*|dx| + 0.040*|dy| + bend + 0.002*log2(fanout)
```

`bend` is 0.005 ns when both coordinate dimensions change. Device loading may
replace these built-in values. Intrinsic combinational arc delay continues to
come from the technology delay table.

For every endpoint, the analysis records estimated arrival time, required
period, slack, and the placed edges on its critical input path. Violating paths
generate attraction forces on both ends of each critical edge. Forces from all
violations are accumulated per cell, with the largest contributor retained as
the cell's primary attraction peer.

`PlaceDesign::refineTiming()` performs up to six bounded passes after initial
packing. It selects at most 64 force anchors per pass. For each anchor it forms
a small constellation of nearby connected cells, excluding the attraction
peer, and moves that local matter one Tile in the common force direction.
Leading cells move first so a translated constellation can open capacity for
the cells behind it. Every candidate still passes the normal `Element` and
Tile-chain legality checks, and fixed cells do not move.

A pass is transactional. Placement timing is recomputed after the proposed
moves; the pass is retained only when total negative slack improves, or when
total negative slack ties and worst slack improves. Otherwise all cells return
to their exact original Tile positions. `PLACE_TIMING_PASS` reports accepted
progress and `PLACE_TIMING_SUMMARY` reports endpoints, evaluated timing nodes
and edges, violations, worst slack, total negative slack, movement counts, and
elapsed time.

Timing traversal is expected to be linear in the timing forest plus its
endpoint edges; shared combinational outputs are cached and driver fanout is
counted once. Ordering the `F` cells that receive forces adds `O(F log F)` work.
Refinement performs at most one full analysis per accepted pass plus the
initial analysis, while candidate packing work is bounded by the anchor and
constellation limits.

#### Timing-driven row and column sorting

An opt-in Sorting-only experiment, `PlaceSortingConfig::chain_center`, visits
every unique cell of the selected critical setup chain, in launch-to-capture
order. Its destination is the arithmetic mean of those cells' Tile coordinates,
including fixed anchors. The mean is frozen for that chain visit and rounded
to the nearest Tile for movement; fixed cells contribute to the mean but never
move. Each movable register or combinational cell uses the rotating-direction
evacuation, half-deficit step calibration, packing preview and forward timing
acceptance. Processing stops if the current endpoint reaches nonnegative slack.
After a successful axis move, Sorting continues through the remaining eligible
directions for that cell, using its new position and refreshed slack. A successful
horizontal move no longer suppresses the vertical attempt (or vice versa).
The chain center and global once-per-cell rotation remain unchanged.
This is a selected critical chain, not every branch of its timing cone, and it
does not change Outline, PlaceDesign, PlaceTiming or PlaceSwapping.

[`PlaceSorting`](../src/pnr/place/PlaceSorting.cpp) runs after refinement and
before swapping. It performs one full timing analysis, creates the DEFICITE
endpoint list, and then uses `PlaceTimingLocal::updateForward()` to update
the existing timing objects after each committed cascade. This forward refresh
does not copy timing endpoints into rollback transactions.

`SCALEPNR_PLACE_SORT_AUDIT=1` is a Sorting-only diagnostic: it records accepted
cell displacements and direction outcomes, prints the six worst final setup
paths, and replays each complete input cone to identify exact slack changes.
`SCALEPNR_PLACE_SORT_AUDIT_ENDPOINTS` optionally adds comma-separated endpoint
names. It scans individual-cell row/column timing optima without packing, then
tests center/optimum targets with actual cascade packing in isolated forked
children. A second diagnostic trial may bypass the affected-path slack guard;
its exact endpoint/WNS/TNS consequences are printed, never committed to the
parent placement. The puzzle exits after this audit without running Swapping.
Instrumentation affects the number of moves fitting the time limit; results
identify this audit run, not a replay of a previous wall-clock-limited run.

Before packing, candidate timing walks every input branch of affected endpoints
directly on the existing timing forest, using proposed Manhattan geometry and
unchanged intrinsic delays. It does not cache predictions or modify live timing.
Checking only the old critical edges is incorrect: a previously shorter branch
can become critical when its launch moves. The scan also stops when even
independently choosing stay/shift for every remaining cell on the selected path
cannot improve that path. No radius or candidate-count limit is introduced.
The candidate must improve the selected endpoint without sacrificing an already
worse endpoint; a degraded neighbor must remain strictly above the selected
endpoint's pre-move slack. There is no TNS or violation-count veto.
If a rejected cone has no movable cell on the unvisited part of the row/column,
extending that cascade cannot change its timing. The scan stops immediately and
tries a shorter displacement; it does not walk unrelated Tiles to the boundary.
Sorting does not save and temporarily apply coordinates
for timing trials. It keeps only source/destination packing metadata and
proposed coordinates for geometry evaluation. Promising plans enter `ElementPackingPreview`,
preserving existing slots where possible and trying the linear slot selector
when a slot is busy.
All previews coexist so connected chains see the proposed occupancy. Displaced
cells are reserved from the boundary inward; A/B is reserved and committed last.
No general combinatorial pack search is used. A successful plan is committed
once at the exact previewed positions through `Tile::tryAddAt()`; failed
previews restore ownership without replaying physical placement. Packing
preflight remains non-destructive; this is not a rollback of a committed shift.
After committing, affected timings are refreshed in place, including newly
critical inputs. Preflight and committed timing use the same all-input model.
Subsequent decisions use the corrected timings. Occupancy
indices are updated only for the affected cells. `PLACE_SORTING_SUMMARY`
reports timing evaluations (one per committed move) and packing previews
separately from candidate plans, and counts only committed displaced cells.

#### Final timing-driven swapping

[`PlaceSwapping`](../src/pnr/place/PlaceSwapping.cpp) consumes only an already
legal packed placement. It rebuilds placement timing, sorts violated endpoints
by slack, and sorts each endpoint's critical edges by wire delay. Setup-deficit
endpoints and setup-proficit challenger bunches are indexed in a configurable
two-dimensional region map. For every critical A-B edge, candidate discovery
is deliberately simple: visit every map region in the axis-aligned A-B
rectangle with nested `y` and `x` loops, then visit the PROFICITE entries in
that region. Before conversion to map regions, the physical rectangle extends
ten Tiles beyond both A and B in X and Y and is clamped to the device. This
gives horizontal and vertical critical edges useful off-axis area. There is no
rasterized-line, supercover, corridor, strip, or direction-tracking search.

Each pass uses one frozen timing analysis and one frozen DEFICITE/PROFICITE
map. DEFICITE always contains every setup endpoint at or below the configured
`-0.1 ns` threshold; its membership is independent of TEMPERATURE. The pass
directly traverses those entries until the list is exhausted or 100
provisional swaps have been accepted. Reaching that configurable cap
immediately finishes candidate traversal and starts exact pass validation.
For one critical edge, every eligible PROFICITE entry in the rectangle is
tested directly and restored. Locally valid choices are ranked by the worst
setup slack across all paths affected by A, B, and C, including incoming and
outgoing paths. Ties prefer the smaller change in total negative slack, then
the better selected endpoint slack, then stable cell order. Using a TNS change
keeps candidates with different affected endpoint sets comparable. This prevents
an oversized improvement of the selected outgoing path from outranking a
balanced candidate which also preserves its incoming timing. The score reuses
the existing local timing evaluation; no extra timing analysis is required.
The best-ranked candidate is committed provisionally after the complete
rectangle traversal, while the remaining order is used only as exact-recovery
fallback. A candidate's expected endpoint improvement is calculated from the
frozen critical path and the translated Manhattan geometry. A rigid-translation
estimate is not sufficient to reject a candidate: the existing packer can move
non-anchor bunch members within `placement_radius`. If rigid translation fails
the improvement threshold, the filter also checks an optimistic wire-delay
bound over that permitted follower movement. The anchor remains fixed at C's
Tile; each follower has its configured Manhattan-radius allowance. The bound
can admit a packing trial, but cannot accept a swap: the same exact A/B/C timing
checks and 5% improvement requirement still apply. The horizontal and vertical
`packing_can_improve_a_rejected_rigid_projection` regressions reproduce a legal
repair rejected by the old filter and verify that C's timing is preserved.
`PLACE_SWAPPING_MOVE` marks these newly admitted repairs with
`packing_projection_rescue=true` and prints the actual packed edge distance.

If all existing translation attempts for an endpoint fail,
`repack_combinational_fallback` (enabled by default) repeats its C searches
with the moving bunch's non-anchor combinational cells seeded at the new
anchor, rather than at their old translated offsets. The same
`placement_radius` bounds their local legal packing. Clocked followers retain
their translated offsets; C's replacement algorithm is unchanged. This avoids
preserving a long LUT-to-register separation created by earlier placement
stages. This fallback can also examine an A–B edge inside one bunch, since
repacking can change its internal geometry; that bunch is tried only once
per C, not once for each endpoint. Existing successful translations retain
priority. Both the geometric filter and exact recovery replay use the
selected relocation mode; all timing acceptance requirements remain the same.
For an internal bunch link, compaction first tries its existing anchor without
requiring a foreign C partner. The ordinary relocation transaction saves that
bunch once, keeps its anchor in place, and repacks its combinational followers.
It uses the same exact packing, affected-cone timing, pass validation, and rollback
as an ordinary swap. Only a failed local repair searches other bunches. This also
works when the PROFICITE map is empty. Four mirrored/oriented regressions verify
the repair, unchanged registers, exact timing, and release of the old LUT slot.

Four mirrored/oriented `displaced_comb_is_repacked_near_its_new_anchor`
regressions verify a known legal repair, unchanged C setup slack, and the
failure of translation alone.

Candidates are existing movable `RegBunch` objects. Before mutation, the
implementation snapshots both groups' exact Tiles, element positions,
physical and outline coordinates, and bunch coordinates. It unassigns both
groups, places the critical bunch at the challenger's former anchor, and uses
the bounded local packing radii to place its followers and the displaced
challenger. Every position is committed through `Tile::tryAdd()`.
Internal-chain net refresh classifies eligible MUX/CARRY sinks once per Tile
update. It then visits the same effective driver/sink pairs in the same order.
This avoids an otherwise quadratic no-op type scan in register/LUT-only Tiles,
including every speculative placement and restoration; packing rules and
internal-chain routing flags are unchanged.

Only after the traversal or its 100-swap cap is timing rebuilt. All swaps made
during the pass are therefore one bounded provisional batch. The complete
batch is retained when the pass-level timing tradeoff is accepted. If the
batch fails, its snapshots first restore the collision-free pass-start state.
Candidate discovery also stops before the hard stage deadline, reserving a
bounded recovery interval. This prevents a late rejected batch from consuming
the entire runtime and leaving no time to retain its individually valid moves.
Recovery considers all locally valid alternatives for each proposal, subject
to the stage timeout. It reevaluates only endpoint cones touched by the moved
cells. A slack multiset and accumulated endpoint slack changes maintain exact
global WNS and TNS without scanning the full design for each candidate. Local
transactions restore endpoint paths as well as timing totals when a trial is
rejected. The best exact WNS result is committed with TNS as a tie-breaker.
There is no geometric shortlist or per-proposal timing-analysis cap. The
selected endpoint's improvement is recalculated against the current recovered
placement. After recovery, one independent full analysis verifies the local
WNS, TNS and violation count and refreshes the force data.
An accepted state is used to rebuild both maps for the next pass. The
implementation also retains the best accepted WNS state, using TNS as a
tie-breaker, and restores any accepted tail after that state before returning.

With `V` deficit endpoints, `E` retained critical edges, `R` regions in an
A-B rectangle, `C` PROFICITE entries per region, and `P` cells in the exchanged
bunches, candidate enumeration is `O(V * E * R * C)` and packing is local in
`P`. Full placement timing is evaluated once for a successful provisional
pass. A rejected batch additionally performs one full verification after local
recovery, independent of rectangle area and the number of candidates.
`PLACE_SWAPPING_MOVE` identifies
provisional exchanges, `PLACE_SWAPPING_PASS` reports batch acceptance or
rollback, and `PLACE_SWAPPING_SUMMARY` reports passes, candidates, packing
failures, pass-level timing analyses, and restored cells.
`PLACE_SWAPPING_LOCAL_TIMING` reports evaluator call count and elapsed time.
The puzzle's optional `SCALEPNR_PLACE_SWAP_COMPARE_EVALUATORS` mode forks two
diagnostic runs from the same packed design and compares reference/prepared
evaluation without regenerating Outline or changing cell order. Like the
existing parameter sweep, this is a diagnostic run, not a puzzle success check.

### Current conformance gaps

The implementation establishes the five-stage structure and most of the
required data flow, but two requirements above are not complete yet:

- Outline records timing deficit and sorts work by it, but does not currently
  scale attraction by deficit because the critical-link multipliers are
  disabled.
- Outline shifts coarse bunches away from estimated LUT overload, but exact
  Placing does not feed a newly discovered Tile-type or chain-capacity failure
  back into a whole-group shift; it falls back to radial per-cell search.

These gaps should be treated as implementation work against the top-level
contract, not as exceptions to that contract.
