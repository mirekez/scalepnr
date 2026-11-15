!!! Never put in these documents any results of tests or their lists/descriptions or results of experiments or unsure things or general talks. These files only for short requirements / explanations !!!

# Place-and-route methodology

## Main principles

Place and routing are a **partial, iterative process**, not two independent
whole-design jobs. Estimate the entire design first, order work by timing
criticality, and admit a bounded part of that ordered work in each **run**.
Complete both placement and routing for that part before admitting the next.
The most critical work gets the first choice of placement and routing resources.

The execution hierarchy is:

```text
Estimate: whole-design timing criticality and bunch formation
run 1: most critical part
  placement stages -> passes
  routing stages   -> passes
  validate and freeze
run 2: next part, around the frozen first part
  placement stages -> passes
  routing stages   -> passes
  validate and freeze
...
final validation: every required cell placed and every required connection routed
```

`SCALEPNR_PNR_RUN_CELLS` is the global default target for newly admitted cells
per run: **1000**, half the 2000-cell target used in the recent example 1
profiling runs. The old compiled default was 7500. Larger 5000–10000-cell
chunks remain configurable, but are not the current default. This is
a work-size target, not a claim that every such design is routable. Keep a bunch
and mandatory local packing chains together; report an oversized indivisible
group explicitly rather than silently losing cells or breaking its chain.
Count real cells, not the number of bunch roots or recursive estimates.

Estimate belongs to PnR as a whole (`src/pnr/EstimateDesign.h/.cpp`), not to
placement. Its criticality formula remains authoritative: larger timing deficit
first, then larger path delay. Selection must cover every physical cell once,
including cells not reachable from a top-level output. Shared cells must not be
counted or admitted twice. Clock domains retain their individual constraints.

`src/pnr/pnr.h` and `src/pnr/pnr.cpp` own run orchestration. Placement and
routing engines continue to own their stages and passes; the orchestrator must
not duplicate their algorithms.

### Place later chunks around existing routing

The purpose of incremental placement is not merely to find unused LUT/REG
slots. A physically empty Tile can contain busy transit routing. Later runs
must consider this routing situation during placement to **reduce the need for
preemption**, and hence the expensive repair of preempted routes. Keep earlier
completed cells fixed during placement. Moving stages may relocate these cells
using angle-guided backward or forward tracing when a connection cannot finish
at its existing position; `outline.fixed` is a placement restriction, not a routing relocation
ban. Preserve physical I/O site constraints. Reopen completion only after a
committed move, update its placement target, and repair all affected routes,
including clocks, before completing the current run.
Both movers select a legal placement before try routing its other inputs, if failed to route - continue placement search. They
then call the same source-moving method recursively for the input drivers,
without broad input-route proofs or per-input search budgets. Already-complete
physical bindings are skipped when their outer queued tasks are encountered.
See [Moving Sources](routing.md#4-moving-sources) and
[Moving Destinations](routing.md#6-moving-destinations) for the algorithm and
ownership rules.
Retain their routes by default, but allow eligible
transit preemption when needed, even on `pnr_preserved` nets. Minimize
preemptions through placement and packing, not a ban on previous-run victims.
Do not weaken preemption validation or skip necessary repair to save time.

Pre-smearing compares the first feasible ring and two more nearby Manhattan
rings when the requested region already carries routes. It prefers lower
pressure, then the existing timing cost and distance rules. Whole bunch shapes
and exact Element previews remain authoritative. The per-cell timing-aware
fallback likewise compares nearby Tiles; shared-input shortcuts may not bypass
this ranking. This is a bounded preference, not a ban on using occupied CBs:
if only such Tiles can pack the cells, placement can still use them.

Internal timing refinement rejects moves into higher-pressure CBs. Sorting
and Swapping may redistribute pressure among the cells they move, but may not
increase their summed pressure. These checks prevent subsequent timing-only
steps from immediately undoing route-aware packing. First-run and standalone
placement retain their existing behavior.

The occupancy score is a cheap congestion heuristic, **not a proof that a
particular input or output can route**, nor a prediction of exact preemption
counts. It does not walk route owners, perform speculative routing, or alter
leases. Routing still proves complete connections. The real example must be
measured before claiming a reduction in preemptions or successful completion.

## Run contract and boundaries

- Clock infrastructure required by admitted cells must be available in that
  run. Constants and generated local-chain helpers belong to their consumers'
  run. Infrastructure must not be recreated or erased between runs.
- A successful run has legal placement and complete routing for all currently
  eligible connections. Report timing as well; a timing allowance must never
  hide an unplaced cell or incomplete route.
- Freeze completed cells using their fixed flag. Later Outline, smearing,
  timing refinement, Sorting, Swapping and routing-driven movement must not
  change their coordinates or packing positions.
- `pnr_preserved` is not preemption protection: validated
  Takeoff/Grounding may cut eligible transit from earlier runs and repair it
  within the current run. Infrastructure protection and endpoint protections
  still apply. All affected connections must be complete before the run succeeds.
  New fanouts may extend an existing source tree without rebuilding it.
- Placement operates over the existing occupied device. Actual Element packing
  compatibility and available capacity remain authoritative. Fully occupied
  candidates can be skipped cheaply; an abstract occupancy count must not
  replace the exact packing check for mixed cells and chains.
- Constant-fed inputs must have an available path from their declared local
  constant root, not merely a free input pin. Check live LOCAL/JOINT owners
  during previews and exact packing; allow sharing with routed constants of
  the same polarity, but not foreign routes. This is local reachability, not
  ordinary incoming-DST demand or permission to preempt Grounding.

## Development plan and verification

1. **Integration.** Expose the full orchestrated flow through Tcl, exercise
   multiple small runs with real placement and routing, and test deferred edges,
   capacity pressure, immutable earlier runs and failure stopping later runs.

## Invocation

After loading the device, RTL and constraints, use:

```tcl
open_design
pnr_design -run_cells 1000
```

`pnr_design` without options uses the CMake default
`-DSCALEPNR_PNR_RUN_CELLS=1000`. The command estimates automatically if no
Estimate forest exists. It returns the number of completed runs on success,
and a Tcl error on failure. The standalone `place_design` and `route_design`
commands remain available for stage diagnostics.
