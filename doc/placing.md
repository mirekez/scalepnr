# Placing

ScalePNR placing has five ordered stages: **Estimate**, **Outline**,
**Placing**, **Sorting**, and **Swapping**. The first two stages preserve
freedom by working with logical groups and continuous coordinates. Placing
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

This precise acceptance test is what permits dense resource use without
creating a placement that is locally impossible. A failed candidate must not
partially consume resources. A successful candidate must atomically commit the
Tile, element position, occupancy, and affected local reservations.

The stage completes only when every placeable cell has a legal assignment.
Failure to place a cell after bounded recovery passes is an explicit placement
failure, not a successful partial result.

### 4. Sorting

Sorting is the first repair stage after exact packing and placement-aware
timing refinement. It builds a list containing only setup endpoints below the
default `-0.100 ns` DEFICITE threshold and visits them in worst-slack order.
For each violated setup path, `A` is its launch driver and `B` its capture
sink. Both receive the same independent correction procedure; fixed endpoints
are skipped. An intermediate LUT on the longest wire must not replace either
timing endpoint.

The relative N/NE/E/SE/S/SW/W/NW positions of the peer determine which row or
column movements bring A/B closer. The other cells must move in the opposite
direction. Among those compatible evacuation directions, Sorting tries the
closest chip boundary first, using actual Tile distances to the device's
north/east/south/west edges, not the longest coordinate difference to the peer.
Equal boundary distances use stable N/E/S/W order. An axis-aligned peer admits
one direction; a diagonal peer admits two. If the absolutely closest boundary
would send A/B away from its peer, it is not eligible.

For example, A=(2,5), B=(3,10) selects westward evacuation (two Tiles to the
boundary) before northward evacuation (five Tiles), while A itself moves east
toward B. The correction never overshoots the peer along the selected axis.
Displaced Tile contents are packed from the vacancy back toward the target,
then A/B is packed last into the freed position. If no correction works along
the first axis, the other compatible axis is tried. Evacuation in the same
direction as A/B is forbidden.

The requested distance is calibrated from half of the endpoint's negative
setup slack because both `A` and `B` receive an independent opportunity to
move. Every cascade uses the normal abstract `Element` packing rules. A move is
committed when the critical-path geometry predicts improvement of the selected
endpoint without a global WNS regression and exact packing succeeds. TNS is
not an acceptance constraint. Sorting is forward-only: it never rolls back a
committed shift after recalculating timing.

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
Placing with placement-aware timing refinement, runs Sorting, and finally runs
Swapping.

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

#### Timing-driven row and column sorting

[`PlaceSorting`](../src/pnr/place/PlaceSorting.cpp) runs after refinement and
before swapping. It performs one full timing analysis, creates the DEFICITE
endpoint list, and then uses `PlaceTimingIncremental::updateForward()` to
recalculate only the setup cones touched by each committed cascade. This
forward refresh does not copy timing endpoints into rollback transactions.

`evacuationDirections()` implements the eight relative peer regions and sorts
their compatible evacuation rays by distance to the chip boundary.
`directionFor()` returns the first such direction.
`estimateShiftTiles()` converts half of the current deficit to a Tile count
using the horizontal or vertical wire-delay calibration. For every direction,
the search first tries the destination itself, then extends a relocation plan
from that destination toward the selected chip boundary, always opposite to
the movement of A/B. Vacancies between the destination and the cell's original
position are included. The complete movable contents of every intervening Tile
shift one Tile toward the vacancy.
A cheap boundary check compares existing plus incoming primitive counts with
the abstract Element position masks before constructing packing previews for
the whole segment. This only rejects impossible counts; shared resources and
chain connectivity still go through exact packing.
A Tile containing a fixed cell cannot be crossed. If the requested displacement
is blocked, shorter corrections are tried before abandoning that direction.

As the plan grows, cached Manhattan-delay deltas update the affected existing
critical paths only for newly appended cells. These paths bound the new arrival
times from below, allowing impossible plans to be rejected without modifying
placement or traversing timing cones. They are the forward acceptance estimate;
the scan also stops when even independently choosing stay/shift for every
remaining cell on the selected path cannot improve that path. No radius or
candidate-count limit is introduced. The estimate must improve the selected
endpoint without predicting a global WNS regression; there is no TNS or
violation-count veto. Sorting does not save and temporarily apply coordinates
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
After committing, affected timings are refreshed in place. A newly critical
input can make the exact result differ from the prediction; even then the shift
is retained, and subsequent decisions use the corrected timings. Occupancy
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
frozen critical path and the translated Manhattan geometry. After
packing a swap, a bunch-to-endpoint index finds only setup paths touching A, B,
or C. The index includes all branches feeding each endpoint, including branches
that were not critical when the pass started. A local evaluation traverses only
these affected timing cones and selects their critical paths again using the
new Manhattan geometry. `PlaceTimingPrepared` compiles the unchanged timing
forest into indexed inputs/outputs once and caches intrinsic delays and fanout
factors. Every trial starts a new evaluation epoch, reads current coordinates,
reconsiders every input, and reconstructs the exact critical path. It does not
reuse arrival times from the previous placement. This removes repeated graph
hashing, connectivity walks, and static delay lookup from candidate evaluation;
full pass validation still uses the independent original timing analysis.
The lifetime of this evaluator must not span connectivity, intrinsic-delay, or
timing-forest changes. `SCALEPNR_PLACE_SWAP_REFERENCE_LOCAL_TIMING` selects the
original evaluator for differential diagnostics. Neither evaluator changes
the 5% improvement threshold, candidate rectangles, candidate ranking, the
100-swap pass cap, cooling, or the stage deadline.
It does not rebuild either map. A single `TEMPERATURE`
parameter controls only the accepted A/B slack floor and C's relative
degradation allowance; it never filters the DEFICITE worklist. Its initial
value is the absolute WNS measured when PlaceSwapping starts. Thus, if WNS is
`-2.7 ns`, the first pass uses `TEMPERATURE=2.7`: an A/B bunch which started
above `-2.7 ns` must remain at or above that floor, an A/B bunch already below
the floor may not deteriorate, and C may lose at most `2.7 ns` relative to its
own pre-swap slack. The selected endpoint must still satisfy the configured
improvement requirement. Temperature cools by a configurable amount after
every pass, moving the A/B acceptance floor toward zero while tightening C's
permitted relative loss by the same amount. The default is `0.1 ns` per pass,
so a `1 ns` initial temperature reaches zero after ten cooling steps. Cooling
stops at zero. Exact pass-boundary validation
repeats the per-bunch timing limits recorded by the provisional swaps. There is
no per-bunch participation limit: a bunch may move repeatedly or be selected
again as C while it satisfies the candidate and timing checks. Previously
visited complete placements are rejected to prevent exact cycles.

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
