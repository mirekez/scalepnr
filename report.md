# scalepnr Place-and-Route Testing Report

**Project:** scalepnr open-source FPGA place-and-route tool  
**Target used for physical validation:** AMD/Xilinx Artix-7 XC7A100T, package `fgg676`, speed grade `-1`  
**Report date:** 5 August 2026  
**Tested random-design campaigns:** 300 designs, seeds 198 through 497

## 1. Subject

This work validates that scalepnr can place and route synthesized FPGA designs
using a hardware-independent internal model and then export enough physical
information for an existing vendor tool to accept exactly the same result. The
immediate physical reference is the Artix-7 XC7A100T database from prjxray-db,
but vendor names and device-specific packing rules remain outside the generic
`src/` routing model. The core objective is therefore stronger than making one
example complete: the same generic placer, crossbar model, route ownership
rules, and route-tree representation must survive many unrelated netlists and
must produce a physically legal result every time.

The internal FPGA model treats the device as a grid of tiles. Every tile may
contain both local resources and routing resources. A crossbar type describes
legal connections among local, destination, source, and joint nodes with
numeric masks. A tile state stores the currently leased nodes. Cross-tile
movement is selected numerically and resolved through `dst_by_src`; runtime
routing does not choose paths by parsing vendor wire names. Tile resource
packing is likewise described by generic element types and connectivity masks.
An architecture adapter may annotate these generic objects with physical names
for export, but that annotation does not control the route search.

The verification problem has four distinct parts:

1. Check isolated invariants, such as mask iteration, route lease ownership,
   branch removal, packing compatibility, docking, preemption, serialization,
   and exact cross-tile destination resolution.
2. Exercise complete placement and routing on generated designs rather than
   only on hand-written examples.
3. Keep the generated combinational depth realistic and reproducible. The
   random generator therefore accepts a maximum chain length, and an
   independent post-synthesis graph check rejects any generated design that
   exceeds it.
4. Submit scalepnr's result to Vivado as explicit placement and fixed-route
   constraints. Vivado must accept every cell location and BEL, accept every
   route tree, finish implementation with zero failed nets and zero node
   overlaps, and export exactly the same physical PIP set.

The random campaign covered three chain-length limits: 5, 10, and 30
combinational primitives between sequential boundaries. Each class used 100
independent deterministic seeds. The campaign found two concrete defects that
were fixed and replayed: relocation of a movable source when its sink is fixed,
and an exporter error that confused an O6-to-primary-FF direct connection with
an O6-to-secondary-5FF connection. After these fixes, all 300 designs passed
the complete Yosys, scalepnr, export, Vivado, and strict comparison flow.

The resulting evidence supports two claims. First, the generic PnR state is
internally consistent under focused regressions and randomized congestion.
Second, for every tested design the architecture adapter translates that state
to an Artix-7 implementation that Vivado recognizes as the same placement and
the same routing. This is a physical implementation result; it does not by
itself claim formal HDL equivalence, timing closure, board-level electrical
correctness, or coverage of FPGA resource classes excluded from synthesis.

<div style="page-break-after: always;"></div>

## 2. scalepnr PnR Approaches, Algorithms, and Efficiency

### 2.1 Hardware-independent physical model

`CBType` is the immutable connectivity model for one crossbar subtype. Its
tables map numeric node identities through legal transitions such as
`local_src`, `local_joint`, `dst_src`, `dst_joint`, `joint_src`, and
`dst_local`. Derived masks accelerate common reachability questions. `CBState`
contains the dynamic leases for the tile. `NodeMask` contains 4,096 bits as 32
128-bit words, so candidate filtering is performed with bitwise operations and
set-bit iteration rather than with runtime string lookup.

A source node is selected from the connectivity mask in directional priority.
The priority is based on the loaded coordinate delta relative to the target;
shorter wires are preferred when angles are equal. After a source bit has been
selected, `dst_by_src` gives the exact target crossbar subtype, coordinate
delta, and destination-node mask. This separation is important: connectivity
masks decide what can be used, while `dst_by_src` resolves where the selected
physical jump lands.

The tile resource model uses the generic element categories `LUT5`, additional
`LUT1`, `MUXF7`, `MUXF8`, `CARRY`, and `FD`. For each category, a 16-bit
position mask states which element lanes exist. Left and right blocker masks
describe which positions in adjacent element columns are physically linked.
`Tile::tryAdd()` selects a free position, follows these neighbor masks, and
allows an occupied neighboring position only when the actual netlist cells are
connected in the required direction. The same recursive relation detects
longer LUT-to-mux-to-register chains. This permits architecture databases to
describe different tile organizations without embedding a vendor tile name in
the packing algorithm.

### 2.2 Outline and detailed placement

The design estimator groups registers and their combinational cones into
`RegBunch` structures. I/O cells are anchored first from package assignments.
The outline phase distributes the remaining groups on a coarse mesh and
iteratively attracts connected groups while limiting local density. Its
optimization count is `max(1, cells / 10)`, making the amount of global
refinement proportional to design size rather than to the full device area.

Detailed placement projects each cell's outline coordinate onto the physical
tile grid. It performs a deterministic radial search from that preferred
coordinate and calls `Tile::tryAdd()` at each candidate. Carry chains prefer
the location vertically adjacent to their placed predecessor. Strict local
chains place their producers before consumers and reserve compatible lanes for
LUT-to-MUXF7 and MUXF7-to-MUXF8 arcs. A cleanup sweep revisits packable cells
deferred by dependency ordering. Generated passthrough elements may be inserted
when a resource is not adjacent to the routing-facing side of the tile; the
corresponding internal connection is represented as a void net rather than as
a fabricated external route.

### 2.3 Generic routing

The first routing stage builds one trunk for each physical source port. Task
collection marks a source port after selecting its first sink, so multiple
fanouts cannot all consume independent exits at the source tile. Constant-zero
and constant-one loads use the same task machinery after `RouteVCC` has
identified their distributed source endpoints.

Each search attempt is deliberately short. A normally five-hop recursive
continuation enumerates direct and joint-mediated exits from the current node.
Speculative suffixes and their leases stay local to the recursion. If the
suffix succeeds, it is appended and leased atomically. If a child fails, the
search returns to its parent and tries the next source bit. Useful partial
progress is committed, and the next bounded pass continues from the committed
endpoint. This avoids paying for one large exhaustive search and keeps the
inner operation dominated by masks, fixed-size node records, and short vectors.

Persistent `src_deadend` masks are active only in Generic routing. They record
failed trunk exits across passes. A direct final hop and the docking search may
ignore these marks because reaching a sink is a different problem from proving
a transit continuation unproductive.

### 2.4 Moving sources

The second stage receives only trunks left by Basic. It disables persistent
deadends, moves each blocked physical driver to a nearby sparse legal placement,
atomically invalidates that driver's source tree, and rebuilds one Generic
trunk. All secondary bindings remain parked. This stage is a hard barrier: it
must finish with zero active and deferred trunks before Fanout work is released.
Generated source passthroughs retain their route endpoint identity, but source
relocation follows their void ownership chain to the physical driver.
The conserved trunk queue receives one global deadend-free retry when Moving
sources starts. Independent unresolved drivers are then relocated in five
bounded batches per design quantum, and each batch shares one Generic retry
pass. Each
candidate must expose a currently free resolved takeoff, including any required
joint resources. Exhausted sources are held in a retry-cycle queue, which
prevents a few congested sources from starving the rest of the stage.

### 2.5 Fanout routing

The third stage starts only after every Generic trunk exists. A Fanout task never
starts another route from the source tile. It follows the existing trunk and
already routed siblings, looking for a transit destination node with a free
outgoing source. A point with more than two available exits is preferred; any
usable point is retained as a fallback. The new binding copies the common
prefix as shared fragments and owns only its private suffix.

Shared ownership is explicit. Removing a route must either release only a
private suffix or atomically invalidate the complete physical source tree.
When a trunk is removed, all source siblings are requeued, one as the new
Generic seed and the rest as Fanouts. Fanout routing ignores persistent Generic
deadends because congestion and ownership have changed, while temporary failed
edges remain active inside the current bounded search.

### 2.6 Moving destinations

The fourth stage resolves suffixes blocked by the current placement. It selects an
unfinished load cell or strict packing cluster, removes only the affected
private suffixes, and tries nearby legal placements in deterministic order.
If the moved cluster contains a physical driver, its source tree is invalidated
atomically; otherwise shared trunks and sibling sinks stay untouched.

For each candidate placement, Moving runs the same two routing modes on the
affected set: Generic first, then Fanout. All incident input routes must finish
before the placement is accepted. Failed candidate coordinates and positions
are remembered to prevent deterministic relocation cycles. Moving also ignores
persistent Generic deadends and uses temporary failed positions during docking
and bounded routing.

### 2.7 Docking and preemption

When ordinary forward routing approaches the sink but cannot reach its exact
local input, grounding docking performs a bounded bidirectional connection.
It expands backward from legal destination entries and forward from the
committed route, using a reverse index built from numeric `dst_by_src` mappings.
Generic CLB docking uses radius 5; edge I/O endpoints use radius 12 with
directional priority. Temporary failed `(coordinate, node)` states prevent the
docking recursion from repeating already disproved branches.

Takeoff preemption is restricted to depth zero when every exit from a source
local is blocked. It may remove a transit route, but not another local endpoint
route. Grounding preemption is even narrower: all physically incoming
destination nodes that can reach the required local must be occupied, the
selected victim must own one of those nodes as transit, and the victim must not
terminate in that tile. Protected infrastructure routes cannot be preempted.

### 2.7 Clocks and route persistence

Dedicated clock routing is performed by `RouteClocks`, outside the three
ordinary stages. It places a compatible clock buffer, routes the input, and
builds a shared output tree to clocked sinks using the same numeric node roles.
Dedicated pass-through wires use numeric `local_by_local` mappings. No
architecture-specific name is consulted during runtime selection.

The design-state database serializes physical source trees, branch endpoints,
typed route edges, shared ownership, local resource pins, and annotation names.
This is sufficient for readback into scalepnr and for an external architecture
adapter to emit physical constraints without changing route decisions.

### 2.8 Observed efficiency

The route iteration limit is `max(16, cells / 10)`, while each top-level stage
has an independent 300-second budget. The measured random campaign shows that
normal runs remain far below that budget. Times below are wall-clock seconds;
the Vivado column includes project generation, placement, fixed-route loading,
routing, export, and strict comparison.

| Chain cap | Designs | scalepnr median | scalepnr 95th percentile | scalepnr maximum without host suspension | Vivado median | Vivado 95th percentile |
|---:|---:|---:|---:|---:|---:|---:|
| 5 | 100 | 74 | 83 | 96 | 67 | 76 |
| 10 | 100 | 77 | 248 | 454 | 68 | 200 |
| 30 | 100 | 81 | 251 | 526 | 70 | 154 |

Two raw timing records include a suspended host: seed 389 recorded 31,655
seconds in scalepnr and seed 431 recorded 31,533 seconds. They completed and
passed after resume, but those wall times are not algorithmic measurements and
are excluded from the normal maximum column.

<div style="page-break-after: always;"></div>

## 3. Testing Techniques

### 3.1 Focused regression testing

The C++ regression suite is integrated with CTest. Tests construct small
devices and netlists directly, use generic/random node names, manipulate exact
mask states, and assert both positive results and absence of collateral state
changes. This is important because a successful route alone does not prove
that another net's lease was preserved or that a failed speculative route left
the crossbar clean.

| CTest name | Source | Main behavior covered |
|---|---|---|
| `fpga.routing` | `src/tests/fpga/routing_test.cpp` | Basic/Moving-sources/Fanout/Moving-destinations task rules and barriers, shared-prefix ownership, branch removal, void-net designators, deadend scope, alternate-exit retry, route completion, and transit-victim selection. |
| `fpga.grounding_preemption` | `grounding_preemption.cpp` | No preemption while a valid destination is free; no endpoint or protected victim; physically reachable entries only; exact transit victim release and requeue. |
| `fpga.moving` | `moving_test.cpp` | Private-suffix invalidation, preservation of large sibling trees, fixed-sink relocation through the movable source, focus ordering, rerouting all incident inputs, and lease cleanup. |
| `fpga.packing` | `packing_test.cpp` | Element position masks, LUT/MUX/FD chains, all lane positions, recursive distant conflicts, connected versus independent packing, and 16-position resource capacity. |
| `fpga.passthrough` | `passthrough_test.cpp` | Generated passthrough creation for source and sink element columns, void internal nets, endpoint mapping, and correct placement beside the owning chain. |
| `fpga.clock_routing` | `clock_routing_test.cpp` | Numeric dedicated-local transitions, exact node identity, compatible buffer placement, source/landing node roles, and exclusion of unrelated dedicated components. |
| `fpga.angle_priority` | `angle_priority.cpp` | Destination-angle priority, shortest line first at equal angle, busy/deadend exclusion, loaded delta precedence, opposite-direction fallback, and randomized masks. |
| `fpga.cb_names` | `cb_names_test.cpp` | Independent local/joint/source/destination name spaces and correspondence between every set connectivity bit and its connection annotation. |
| `fpga.dst_by_src` | `dst_by_src_test.cpp` | Exact selected-source to target-destination mapping, per-step pass-through chains, subtype-specific deltas, and no synthesized destination encoding. |
| `fpga.subtypes` | `subtype_test.cpp` | Construction of Artix-7 routing subtypes from the real database and reverse verification that every numeric connection has a responsible database record. |
| `fpga.docking` | `docking_test.cpp` | Random occupied neighborhoods, backward/forward docking, destination already reached but wrong rail, CLB and I/O radii, joint paths, temporary failures, and complete intermediate-node retention. |
| `fpga.backwards_resolve` | `backwards_resolve.cpp` | Random `dst_by_src` graphs, reverse-index completeness, duplicate elimination, coordinate windows, filtering, and exact scan/reach counters. |
| `fpga.pnr_db` | `pnr_db_test.cpp` | Round-trip serialization of placement, endpoints, trees, shared branches, typed edges, names, and cleanup before readback. |
| `fpga.device_format` | `device_format_test.cpp` | Parsing device/tile formats and preserving resource and routing metadata needed by later stages. |
| `fpga.outline` | `outline_test.cpp` | Design-size iteration limit, mesh indexing, radial coverage, and outline placement boundary behavior. |
| `fpga.arena` | `arena_test.cpp` | A generic 20-by-20 routing arena with randomized load, 50 routes, edge I/O, congestion, directional escape paths, and no vendor-specific topology names. |
| `fpga.repair_prefixes` | `repair_prefixes.cpp` | Shared-prefix consistency repair, ownership transfer, nested descendants, cross-source conflicts, protected trees, and scheduler timeout accounting. |

The final CTest execution completed all 17 tests in 90.39 seconds. The real
database subtype test accounted for 81.04 seconds; all other tests together
completed in approximately nine seconds. The Python exporter suite adds 34
tests for route-tree rendering, canonical physical nodes, I/O endpoint chains,
reserved-node repair, packing legality, source replacement, and same-site
endpoint classification. All 34 passed.

### 3.2 Random design generation

The random tests use the external `pnr_tests` Chisel project. The local wrapper
`random_pipeline.scala` seeds `NodeFabric` explicitly, emits SystemVerilog, and
generates package-pin constraints from an independent deterministic random
stream. A seed therefore reproduces both logic and I/O assignments.

The generator was extended with `maxChainLength`. For finite limits it inserts
one-entry queues between generated processing modules. For the tight cap of 5,
it excludes generator forms with an irreducible deep implementation and keeps
shallow operation subtypes. These generation rules reduce expected depth, but
they are not trusted as the checker.

After Yosys synthesis, `check_chain_length.py` reads the JSON netlist and builds
a producer graph from output bits to cells. Traversal stops at sequential
`FD`, `LD`, `RAM`, and `SRL` boundaries. I/O and clock buffers are transparent;
each remaining primitive contributes one level. Memoized depth-first traversal
returns the longest path, rejects combinational cycles, writes the measured
length, and fails the iteration if the configured cap is exceeded.

### 3.3 Complete random-test pipeline

Each iteration of `.run_random_test.sh` performs the following stages:

1. Generate a seeded Chisel design and XDC-like package constraints.
2. Run Yosys 0.52 with `synth_xilinx -arch xc7`, producing JSON for scalepnr
   and EDIF for Vivado. BRAM, LUTRAM, DSP, and carry inference are disabled in
   this campaign so the tested primitive set matches the implemented model.
3. Measure post-synthesis combinational depth and fail on a cap violation.
4. Add a 5.0 ns clock constraint when sequential cells exist.
5. Run scalepnr, load XC7A100T tile, tile-connection, package, crossbar, and tile
   resource specifications, then place, route, and write `design_state.db`.
6. Run `db2prj.py`. It maps abstract packed elements to Artix-7 sites/BELs,
   converts each physical source tree into one Vivado route tree, writes I/O,
   placement, and routing Tcl, and stages the same EDIF netlist.
7. Run Vivado 2024.2 on `xc7a100tfgg676-1`: link the EDIF, apply I/O and exact
   cell constraints, execute `place_design`, verify LOC/BEL readback, apply all
   routes, and execute `route_design`.
8. Export Vivado's implemented primitive placement and physical PIPs.
9. Run `compare_pnr.py --strict`. Any missing route group, skipped/disconnected
   route marker, placement mismatch, PIP-set difference, fixed-route failure,
   critical warning, failed net, or node overlap fails the iteration.

The script records iteration, seed, requested and measured chain lengths,
SystemVerilog SHA-256, failing stage, and stage times in `summary.tsv`. A
failure keeps all artifacts and prints a one-command replay containing the
exact seed and chain cap. Successful bulky artifacts are normally deleted to
bound disk usage; the compact summary remains.

### 3.4 Independent physical validation

The exporter does not ask Vivado to find an equivalent route. Every generated
route group is assigned as `FIXED_ROUTE` or as a fixed route tree, and
`IS_ROUTE_FIXED` is asserted. Placement is similarly constrained by `LOC`,
`BEL`, and the relevant pin locks. Vivado still constructs its own physical
device graph and validates every requested edge, node, site, BEL, and conflict.
After routing, comparison is based on a set of physical PIP feature names, not
on logical net names; this avoids false differences caused by Yosys/Vivado net
renaming.

The software versions used were Yosys 0.52, Vivado 2024.2 build 5239630,
Scala CLI 1.15.0 with Scala 2.13.12/Chisel 6.5.0 for this source, Python 3.14.5,
CMake 3.26.4, and GCC 15.2.0. The checked source revisions were
`pnr_tests` `cc883e3274a388c3714f375067dbfd8501fa9354`, prjxray
`c9f02d8576042325425824647ab5555b1bc77833`, and prjxray-db
`0a0addedd73e7e4139d52a6d8db4258763e0f1f3`.

<div style="page-break-after: always;"></div>

## 4. Testing Reports

### 4.1 Random campaign summary

The requested campaign used `SIZE=1`, generator complexity 1, maximum width
64, and 100 seeds per chain class. `SIZE=1` is the number of generated
processing stages, not the post-synthesis primitive count; one generated stage
can expand to hundreds of LUTs, registers, and muxes.

| Campaign | Seeds | Requested cap | Measured chain range | Designs passing chain check | Designs passing scalepnr | Designs exactly reproduced by Vivado |
|---|---:|---:|---:|---:|---:|---:|
| Short chains | 198-297 | 5 | 2-4 | 100/100 | 100/100 | 100/100 |
| Medium chains | 298-397 | 10 | 0-10 | 100/100 | 100/100 | 100/100 |
| Long chains | 398-497 | 30 | 0-18 | 100/100 | 100/100 | 100/100 |
| **Total** | **198-497** | **5/10/30** | **0-18** | **300/300** | **300/300** | **300/300** |

Measured chain-length distributions show that the checker exercised more than
one fixed topology:

| Measured length | Cap 5 count | Cap 10 count | Cap 30 count |
|---:|---:|---:|---:|
| 0 | 0 | 10 | 10 |
| 1 | 0 | 25 | 14 |
| 2 | 12 | 3 | 5 |
| 3 | 72 | 33 | 44 |
| 4 | 16 | 3 | 4 |
| 5 | 0 | 4 | 2 |
| 6 | 0 | 6 | 7 |
| 7 | 0 | 1 | 2 |
| 8 | 0 | 4 | 3 |
| 9 | 0 | 9 | 3 |
| 10 | 0 | 2 | 0 |
| 12 | 0 | 0 | 1 |
| 13 | 0 | 0 | 2 |
| 14 | 0 | 0 | 2 |
| 18 | 0 | 0 | 1 |

### 4.2 Stage timing report

| Chain cap | Generation median/max | Yosys median/max | Chain check median/max | scalepnr median/95th/normal max | Vivado median/95th/max |
|---:|---:|---:|---:|---:|---:|
| 5 | 9 / 28 s | 3 / 3 s | 0 / 1 s | 74 / 83 / 96 s | 67 / 76 / 131 s |
| 10 | 10 / 20 s | 3 / 5 s | 0 / 1 s | 77 / 248 / 454 s | 68 / 200 / 249 s |
| 30 | 10 / 29 s | 3 / 5 s | 0 / 1 s | 81 / 251 / 526 s | 70 / 154 / 269 s |

The spread is caused mainly by synthesized structure and routing congestion,
not just chain depth. For example, seed 394 has measured depth 3 but produces
975 primitives and required 454 seconds in scalepnr. It is therefore a more
useful stress case than its chain length alone suggests.

### 4.3 Exact retained design structures

The original campaign summary intentionally retained compact metadata and
deleted successful multi-hundred-megabyte run directories. Consequently,
per-cell and per-fragment counts cannot be reconstructed for every deleted run
without rerunning it. The following table contains every unique campaign seed
for which a diagnostic `design_state.db` and synthesized JSON were retained.
The final PASS status comes from the successful replay of the same deterministic
seed and cap.

| Cap | Seed | Chain | Synthesized cells | Yosys named nets | DB logical nets | Physical source trees | Sink branches | Wire fragments | Unique tree edges | Final strict result |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 5 | 219 | 3 | 117 | 36 | 42 | 61 | 213 | 6,410 | 6,385 | PASS |
| 10 | 301 | 6 | 200 | 92 | 114 | 136 | 565 | 19,421 | 8,316 | PASS |
| 10 | 312 | 1 | 157 | 18 | 6 | 192 | 335 | 10,366 | 5,863 | PASS |
| 10 | 394 | 3 | 975 | 193 | 226 | 1,332 | 4,037 | 110,842 | 51,497 | PASS |
| 30 | 399 | 14 | 200 | 66 | 51 | 192 | 557 | 13,220 | 8,591 | PASS |
| 30 | 408 | 3 | 224 | 143 | 94 | 183 | 568 | 15,389 | 11,390 | PASS |

`Physical source trees` counts the architecture-neutral route-tree objects in
the database. `Sink branches` counts all source-to-sink bindings, including
fanouts that share a prefix. `Wire fragments` is the sum of serialized branch
fragments and therefore repeats shared prefixes. `Unique tree edges` counts the
deduplicated typed edges stored by each physical tree. These metrics describe
different layers and should not be expected to equal the logical net count.

The primitive composition of the same retained synthesized designs is:

| Cap/seed | IBUF | OBUF | BUFG | FDRE | INV | LUT1 | LUT2 | LUT3 | LUT4 | LUT5 | LUT6 | MUXF7 | MUXF8 | Total |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5/219 | 36 | 34 | 0 | 0 | 0 | 3 | 1 | 4 | 4 | 14 | 11 | 10 | 0 | 117 |
| 10/301 | 36 | 34 | 0 | 0 | 0 | 0 | 3 | 7 | 5 | 54 | 28 | 32 | 1 | 200 |
| 10/312 | 36 | 34 | 1 | 64 | 0 | 0 | 22 | 0 | 0 | 0 | 0 | 0 | 0 | 157 |
| 10/394 | 36 | 34 | 1 | 675 | 1 | 0 | 4 | 5 | 9 | 14 | 195 | 1 | 0 | 975 |
| 30/399 | 36 | 34 | 1 | 54 | 0 | 2 | 0 | 2 | 3 | 15 | 31 | 17 | 5 | 200 |
| 30/408 | 36 | 34 | 1 | 28 | 4 | 18 | 5 | 20 | 12 | 21 | 26 | 19 | 0 | 224 |

### 4.4 Detailed strict comparison example

Seed 394 exposed the final exporter defect and is the largest retained case.
The `.db` correctly contained external branches from `C6LUT` to `C5FF` and
from `D6LUT` to `D5FF`, but the exporter omitted them as if they were dedicated
same-site connections. The fix requires the LUT output class and FF class to
match: O6 may use the primary `*FF` direct path, while O5 corresponds to the
secondary `*5FF` position. The external O6-to-5FF branches are now preserved.

After the fix, the strict seed-394 replay reported:

| Check | scalepnr | Vivado | Difference |
|---|---:|---:|---:|
| Primitive placements | 975 | 975 | 0 |
| Physical PIPs | 34,418 | 34,418 | 0 |
| Common physical PIPs | 34,418 | 34,418 | 0 |
| Route groups constrained | 1,329 | 1,329 | 0 skipped |
| Simple fixed routes | 828 | accepted | 0 failures |
| Fixed route trees | 501 | accepted | 0 failures |
| Final failed nets | 0 | 0 | 0 |
| Final node overlaps | 0 | 0 | 0 |
| Placement LOC/BEL mismatches | 0 | 0 | 0 |

Logical net names differ between scalepnr/Yosys and Vivado because Vivado may
rename or merge net objects. The strict criterion deliberately compares the
union of physical PIPs and independently checks every generated route group.
Thus net-name differences do not hide a physical difference.

<div style="page-break-after: always;"></div>

## 5. Why Vivado Validation Establishes Bitstream-Eligible PnR

Vivado is not used as a second router that is allowed to replace scalepnr's
work. The test imports the same EDIF netlist, fixes every exported primitive to
the scalepnr-selected site and BEL, applies LUT pin locks where required, and
loads every route group as a fixed simple route or fixed route tree. Vivado's
physical database then validates the requested objects against the exact
XC7A100T device model.

This validation is precise for several reasons:

1. An invalid site, package endpoint, BEL, or pin mapping is rejected while
   constraints are applied or during `place_design`.
2. A route node that does not exist, is not downhill from the preceding node,
   has the wrong canonical identity, or is unreachable from the driver causes
   `FIXED_ROUTE` to fail.
3. Two nets cannot legally own the same exclusive physical node. Vivado reports
   the conflict or a node overlap.
4. A disconnected tree or a route that does not reach every sink appears as a
   failed or partially routed net.
5. The generated Tcl checks the final cell LOC and BEL values rather than
   assuming the constraints were honored.
6. After `route_design`, the test exports Vivado's actual PIPs and compares
   their set exactly with the PIPs exported from scalepnr. Zero difference
   proves that Vivado did not silently repair, remove, or replace routing.

The routed Vivado design is the physical implementation database consumed by
the downstream bitstream writer. Once all placements and routes are legal,
fixed, overlap-free, and complete, bitstream generation does not need to invent
a different placement or route. Therefore the campaign establishes that the
tested scalepnr PnR results are suitable for inclusion in a bitstream for the
same part, package, netlist, and constraints.

The guarantee must be stated at the correct boundary. The current automated
comparison ends after successful `route_design` and exact physical export; it
does not invoke `write_bitstream`. It proves physical placement/routing
legality and exact reproduction, which are the PnR prerequisites for a `.bit`
file. A literal end-to-end bit-file guarantee additionally requires running
final bitstream DRC and `write_bitstream`. It also does not prove functional
equivalence of the source HDL, timing closure at 5 ns, signal-integrity or
voltage correctness on a board, or support for BRAM/DSP/carry configurations
that were intentionally disabled in this campaign.

Within that scope, the evidence is strong: 300 independently generated
netlists, three depth regimes, varied primitive mixes, exact package I/O,
shared route trees, clocks, constants, and congestion-driven relocation were
all accepted by Vivado with no final failed nets, no overlaps, no missing route
constraints, no placement mismatch, and no physical PIP difference.

## Reproduction References

- Random campaign driver: `tests/prjxray/.run_random_test.sh`
- Random Chisel wrapper: `tests/prjxray/random_pipeline.scala`
- Post-synthesis chain checker: `tests/prjxray/check_chain_length.py`
- Reproducible `pnr_tests` patch: `tests/prjxray/pnr_tests_max_chain.patch`
- scalepnr-to-Vivado exporter: `tests/prjxray/db2prj.py`
- Strict comparison: `tests/prjxray/compare_exported.sh` and
  `tests/prjxray/compare_pnr.py`
- Generic routing rules: `doc/routing.md`
- Per-seed campaign records: `tests/prjxray/random_runs/*/summary.tsv`
