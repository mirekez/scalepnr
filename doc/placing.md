# Placing

ScalePNR placing has exactly three ordered stages: **Estimate**, **Outline**,
and **Placing**. The first two stages preserve freedom by working with logical
groups and continuous coordinates. Only the final stage commits cells to exact
device resources.

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

## Main Principles

The data becomes progressively more concrete:

```text
RTL connectivity and timing
        -> forest of timing-aware bunches
        -> continuous 2D bunch and cell outline
        -> exact Tile and element position
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
`Tech::placeDesign()` fixes assigned I/O cells, runs Outline, and then runs the
exact Placing stage.

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
coordinates and then falls back to site order.

#### Bunch distribution and attraction

`optimizeOutline()` counts reachable cells and chooses a bunch iteration limit
of at least one and otherwise approximately one iteration per ten cells. It
also calculates the average LUT capacity of a coarse mesh box. A second,
finer occupancy grid has twice the physical device width and height.

`recurseRadialAllocation()` supplies the initial cyclic distribution. Starting
at the upper-left logical coordinate used by the code, it walks the perimeter
down, right, up, and left. Child bunches continue from their parent's next
perimeter coordinate, while fixed bunches retain their assigned positions.

Every bunch pass calls `recurseSecondaryLinks()`. A secondary link represents
a connection to a bunch whose primary ownership lies elsewhere. If the linked
bunches are more than one mesh step apart, `attractBunch()` pulls both trees
toward one another. The attraction recursively moves parents and children, so
the operation shifts a connected group rather than only one point. Fixed
bunches do not move. Step size changes between phases from `0.1` to `0.05`, and
then to `0.01` while density refinement is active.

After iteration 100, `recurseStatsDesign()` rebuilds per-box register, LUT, and
bunch statistics. When a box exceeds the computed average LUT capacity,
`optimizeOutline()` selects a least-loaded box in the searched half of the mesh
and pulls whole bunches out of the overloaded box until the excess estimate is
removed. Large combinational bunches are also biased toward the mesh center in
this phase.

The estimated link deficit and delay are available in
`recurseSecondaryLinks()`, but the current distance multipliers for critical
links are commented out. Consequently, current bunch attraction is driven by
secondary connectivity and density, not yet by the required timing-deficit
weight. The sorted input order still retains some timing priority, but it is
not a substitute for weighted attraction.

#### Cell smearing

`recurseInstAllocation()` initially puts the cells of each bunch at the rounded
bunch center. `recurseInstPrepare()` caches connected peers in
`optimization_peers` and offsets cells toward peers in other bunches so that
the initial point mass is already opened slightly.

`recurseOptimizeInsts()` performs bounded spreading passes over the fine
occupancy grid. When more than one cell occupies a fine-grid point, a cyclic
eight-count schedule tests right, down, left, and up and moves the cell one
fine-grid step when that point is less occupied and remains inside the bunch's
area. `attractInst()` then pulls connected cells toward one another and
recursively propagates a decreasing half-step through connected peers. Fixed
cells are excluded from movement. Coordinates are clamped to the logical mesh.

The instance iteration count is capped after the first 50 iterations to at
most one additional pass per maximum physical-grid dimension. Outline prints
`OUTLINE_PROGRESS` for both bunch and instance phases and finishes with an
`OUTLINE_SUMMARY` containing cell and iteration counts and elapsed time.

### Placing implementation

Exact placement is implemented by
[`PlaceDesign`](../src/pnr/place/PlaceDesign.cpp). The abstract element and
Tile acceptance model is implemented by
[`Element`](../src/fpga/Element.h) and [`Tile`](../src/fpga/Tile.cpp).
`Element.h` is the architecture-independent packing model used to describe a
Tile's available cell positions and the chains that may occupy them; it is not
only a primitive-type enumeration.

#### Candidate preparation and spatial search

`preparePlaceCandidates()` divides the physical device into the same 10-by-10
regions used by Outline. For every abstract element type and region it builds a
list of only those Tiles whose `TileType` contains that element. A rotating
cursor prevents every request from restarting at the first Tile.

For an ordinary unanchored cell, `tryAddNear()` converts the outline coordinate
to a physical origin region and searches all coarse regions radially. It skips
incompatible Tile types, removes a Tile from a type's candidate list once that
element type is exhausted, and asks `Tile::tryAdd()` to perform the exact
legality check. Thus rare resource types do not cause a scan of every unrelated
Tile.

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

Unanchored cells use the region candidate lists. Anchored cells use a bounded
physical radial search, with a strict same-Tile anchor restricted to one trial.
Search coverage is sized to cover the device rather than using an arbitrary
small radius. If no legal candidate exists, the current code prints the cell
and terminates placement.

After the forest walk, `placeDesign()` makes up to 64 cleanup sweeps over every
packable cell in the hierarchy. Each sweep retries unplaced logic and prints
`PLACE_SWEEP` counts. It stops when all such cells are placed or when a sweep
makes no progress. Long searches report `PLACE_PROGRESS` once per minute, and
the stage writes `place_output.png` for inspection.

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
