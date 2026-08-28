# Placing

ScalePNR placing has four ordered stages: **Estimate**, **Outline**,
**Placing**, and **Swapping**. The first two stages preserve freedom by working
with logical groups and continuous coordinates. Placing commits cells to exact
device resources, and Swapping performs bounded timing repair on that legal
packed result.

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

This precise acceptance test is what permits dense resource use without
creating a placement that is locally impossible. A failed candidate must not
partially consume resources. A successful candidate must atomically commit the
Tile, element position, occupancy, and affected local reservations.

The stage completes only when every placeable cell has a legal assignment.
Failure to place a cell after bounded recovery passes is an explicit placement
failure, not a successful partial result.

### 4. Swapping

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

An endpoint whose slack is at least `-0.100 ns` is within the default timing
tolerance. Its exact negative slack remains visible in timing reports, but it
is not an actionable swapping violation and is accepted by the final placing
closure check. This prevents the repair stage from spending bounded trials on
small calibration and modeling margins.

## Main Principles

The data becomes progressively more concrete:

```text
RTL connectivity and timing
        -> forest of timing-aware bunches
        -> continuous 2D bunch and cell outline
        -> exact Tile and element position
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
- Dedicated local chains are placed as units even when their cells belong to a
  larger timing bunch.
- All iterative searches and spreading passes are bounded and report progress
  on large designs.
- The model remains architecture-independent. Device data describes abstract
  element types, positions, and connections; placing algorithms do not depend
  on third-party architecture names.

## Current Implementation

The current flow is connected in
[`Tech.cpp`](../src/tech/Tech.cpp). `Tech::openDesign()` runs Estimate, while
`Tech::placeDesign()` fixes assigned I/O cells, runs Outline, runs exact
Placing with placement-aware timing refinement, and finally runs Swapping.

### Estimate implementation

Estimate is implemented by
[`EstimateDesign`](../src/pnr/estimate/EstimateDesign.cpp), with its persistent
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

Outline is implemented by
[`OutlineDesign`](../src/pnr/outline/OutlineDesign.cpp); small coordinate and
iteration helpers are in [`OutlineGrid`](../src/pnr/outline/OutlineGrid.h).
It uses a fixed 10-by-10 logical mesh, independent of the physical device
dimensions.

#### Fixed I/O anchors

`placeIOBs()` walks the bunch forest and `placeInstIOBs()` provides a fallback
walk over the instance hierarchy. Package assignments are resolved to a device
Tile and modeled site position. The cell is assigned immediately, and its
outline and owning bunch are marked fixed on the corresponding mesh boundary.
`packageSitePosition()` first matches relative physical and modeled site
coordinates and then falls back to site order. Pin names and package Tile names
are indexed once before the walk, so anchoring is linear in the number of I/O
cells. The port resolver handles both input-side driver connections and
output-side reverse peers before using the legacy instance-name fallback.

#### Bunch distribution and attraction

`optimizeOutline()` counts reachable cells and chooses a bunch iteration limit
of at least one and otherwise approximately one iteration per ten cells, capped
at 251 passes after all attraction phases have run. It also calculates the
average LUT capacity of a coarse mesh box. A second, finer occupancy grid has
twice the physical device width and height.

`recurseRadialAllocation()` supplies the initial cyclic distribution. Starting
at the upper-left logical coordinate used by the code, it walks the perimeter
down, right, up, and left. Child bunches continue from their parent's next
perimeter coordinate. Before allocation, a bottom-up guide records the centroid
and average graph depth of the fixed I/O descendants under every branch. A
fixed bunch retains its assigned position; each non-fixed descendant is seeded
one proportional step from its parent toward those fixed targets. Thus a branch
starts stretched between its output and input anchors instead of stacking all
of its cells at either package edge. Branches without another fixed descendant
retain the cyclic perimeter fallback.

Every bunch pass calls `recurseSecondaryLinks()` for both primary timing links
and secondary links whose primary ownership lies elsewhere. If linked bunches
are more than one mesh step apart, `attractBunch()` pulls both trees toward one
another. Links consuming at least 75% and 95% of their clock period receive
additional pulls. The attraction recursively moves parents and children, so
secondary-link operations can shift a connected group rather than only one
point. Primary tree links pull their endpoints without recursively revisiting
the whole tree; this keeps work linear at high-fanout junctions. Fixed bunches
do not move. Step size changes between phases from `0.1` to `0.05`, and then to
`0.01` while density refinement is active.

After iteration 100, `recurseStatsDesign()` rebuilds per-box register, LUT, and
bunch statistics. When a box exceeds the computed average LUT capacity,
`optimizeOutline()` selects a least-loaded box in the searched half of the mesh
and pulls whole bunches out of the overloaded box until the excess estimate is
removed. Large combinational bunches are also biased toward the mesh center in
this phase.

#### Cell smearing

`recurseInstAllocation()` initially puts the cells of each bunch at the rounded
bunch center. `recurseInstPrepare()` caches connected peers in
`optimization_peers` and offsets cells toward peers in other bunches so that
the initial point mass is already opened slightly.

`recurseOptimizeInsts()` performs bounded spreading passes over the fine
occupancy grid. When more than one cell occupies a fine-grid point, a cyclic
eight-count schedule tests right, down, left, and up and moves the cell one
fine-grid step when that point is less occupied and remains inside the bunch's
area. The separate timing-attraction pass snapshots every cell position before
calculating motion. Each clocked cell then sums its own connected-cell vectors,
including their physical distance and timing-pressure weight, and all calculated movements are
published simultaneously. A fixed I/O does not move; it gives its immediate
connected follower a vector toward the package anchor. Fixed-I/O tension
propagates at half strength per connection step for at most four combinational
steps. A register is a new force origin rather than a dragged follower, so it
recalculates its personal response from the changed neighboring constellation
on the next frozen pass. Fixed boundaries always stop propagation. Thus combinational cells
follow their neighboring stars but do not start independent gravity. No
occupancy veto is applied after force calculation, but every movable cell stays
inside the physical window of its already-spread bunch. This preserves the
coarse spreading solution while local timing gravity shapes the constellation;
without the window a large connected component can contract onto its package
anchors and form a false perimeter ring.

The instance iteration count is capped after the first 50 iterations to at
most one additional pass per maximum physical-grid dimension. Outline prints
`OUTLINE_PROGRESS` for both bunch and instance phases and finishes with an
`OUTLINE_SUMMARY` containing cell and iteration counts and elapsed time.

There is no component-wide centroid, peer-average relaxation, or shared
constellation direction after these passes. Such a step would replace each
register's local timing decision with a graph-wide force and can create false
edge or corner bias. `OUTLINE_SUMMARY` reports the number of locally applied
timing-attraction movements together with directed-edge, iteration, and
elapsed-time statistics.

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
around it. The search never considers a farther ring while a legal candidate
exists in a nearer one. Within the nearest ring, candidates are ranked by
calibrated horizontal, vertical, and bend delay to every connected cell.
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

#### Final timing-driven swapping

[`PlaceSwapping`](../src/pnr/place/PlaceSwapping.cpp) consumes only an already
legal packed placement. It rebuilds placement timing, sorts violated endpoints
by slack, sorts their critical edges by wire delay, and selects the dominant
horizontal or vertical axis for each separated pair. Its configurable defaults
are a five-Tile-wide strip, ten Tiles of search in either axial direction, a
five-Tile follower-repacking radius, a ten-Tile challenger replacement
radius, a five-percent maximum challenger timing degradation, a five-percent
minimum endpoint improvement, an 80% strong-improvement threshold with at most
5% global regression, a 0.100 ns accepted negative-slack tolerance, forty-eight
traversal passes, and at most 32 trial relocations per pass.

The current global-WNS endpoint receives an expanded nineteen-Tile-wide strip
and twenty-six Tiles of axial search in either direction. Timing is rebuilt after
every accepted exchange, so this larger geometry follows the current WNS path
instead of multiplying the search cost and disruption across every violation.

Candidates come from existing movable `RegBunch` ownership. A candidate must
lie in the selected strip. Because the critical bunch moves as a complete
unit, predicted bunch-anchor distance is the primary rank. Predicted distance
between the actual translated critical cells is a secondary rank and diagnostic, not a
hard rejection: another edge in the same endpoint path may outweigh a locally
longer edge. Before mutation, the implementation snapshots both groups' exact
Tiles, element positions, physical and outline coordinates, and bunch
coordinates. It unassigns both groups and places the critical anchor exactly at
the challenger's former anchor. The displaced challenger is handled by a
second search: a timing-weighted centroid of all its external peers supplies
the seed, and legal anchor Tiles are ranked inside the configured replacement
radius. Its followers are translated relative to the replacement anchor that
was actually selected and repacked inside their smaller local radius. All
candidate positions are committed only through `Tile::tryAdd()`.

Every local search has a prefix-stable core. The complete three-Tile follower
window and five-Tile challenger window retain their timing-cost order. Swapping
runs that core scope until a complete pass accepts nothing, then restores the
best-WNS core state before enabling the larger five- and ten-Tile limits for the
remaining passes. Additional Manhattan rings therefore cannot divert the core
trajectory before it is exhausted. The expanded phase starts from the result
the smaller scope would have returned, and final best-state rollback prevents
the added possibilities from degrading that preserved result.

Before the proposal and after both bunches are legal, calibrated wire delays
are summed over the challenger's external bunch boundary. A proposal whose
relative challenger delay degradation exceeds five percent is restored before
running the more expensive full timing analysis. This local guard protects the
logic displaced by the critical repair while still allowing it to improve or
move laterally without requiring a literal exchange of the two regions.

After each legal trial, `PlaceTiming` is run again. A trial is retained only
when the selected endpoint deficit improves by at least the configured
fraction. Normally global TNS and worst slack must remain non-regressive. If
the endpoint deficit improves by at least 80%, a bounded relaxed rule permits
up to 5% regression of either global metric. That bound is a non-compounding
envelope around the timing at entry to the swapping stage: a move may give
back an intermediate improvement while remaining better than stage entry, but
successive relaxed moves cannot ratchet the result below the 5% entry bound.
A violated endpoint bunch may repair itself twice, but a passive challenger
must not have participated in an earlier accepted exchange. Complete placement
fingerprints are remembered, so the
relaxed allowance cannot repeatedly undo and redo swaps. A
rejected trial restores exact element positions through `Tile::tryAddAt()`;
restoration failure is an assertion because continuing from a partially
restored packing would corrupt later trials. `PLACE_SWAPPING_MOVE` identifies
every committed exchange, `PLACE_SWAPPING_PASS` summarizes each complete
traversal, and `PLACE_SWAPPING_SUMMARY` reports executed and improving passes,
candidates, attempts, relaxed acceptances, reused-bunch skips, packing
failures, timing rejections, restored cells, and before/after timing.

Relaxed swaps may cross temporary WNS regressions during the search. The
implementation therefore retains the best-WNS accepted state, using TNS as a
tie-breaker, and reverses the accepted tail after that state before returning.
A later aggregate TNS improvement cannot silently discard an earlier WNS win.

With `V` violated endpoints, `E` critical edges per endpoint, `C` bounded
challengers, and `P` cells in the two exchanged bunches, each attempted
exchange performs bounded local packing plus one placement-timing analysis.
A Tile-indexed bunch map restricts challenger discovery to the selected strip;
it does not rescan every bunch for every critical edge. Defaults inspect at
most 32 violated endpoints, three critical edges per endpoint, two challengers
per fixed two-Tile axial search band, forty-eight passes, and 32 base full timing
trials per pass. Candidate ordering has a stable design-cell tie-breaker, and
bands are traversed breadth-first across endpoints, edges, and both sides of a
swap. Each additional band adds four bounded trials after the complete inner
budget. Extending a stripe therefore appends both candidates and trial budget
without evicting, reordering, or starving the opportunities retained by a
shorter stripe. A pass stops early
when it cannot accept another exchange, so these are upper bounds rather than
mandatory work. These hard limits keep the residual repair stage small
relative to the main placer.

### Current conformance gaps

The implementation establishes the four-stage structure and most of the
required data flow, but three requirements above are not complete yet:

- Outline records timing deficit and sorts work by it, but does not currently
  scale attraction by deficit because the critical-link multipliers are
  disabled.
- Outline shifts coarse bunches away from estimated LUT overload, but exact
  Placing does not feed a newly discovered Tile-type or chain-capacity failure
  back into a whole-group shift; it falls back to radial per-cell search.
- The cleanup sweeps diagnose a no-progress state, but the final sweep itself
  does not assert that no placeable cells remain after it breaks.

These gaps should be treated as implementation work against the top-level
contract, not as exceptions to that contract.
