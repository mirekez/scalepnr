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
per run: **7500**, within the empirically useful 5000–10000-cell range. This is
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

## Run contract and boundaries

- A pending cell must not accidentally get placed or routed by recursive stage
  traversal. The full RTL stays intact for timing estimation and connectivity.
- A connection becomes routing work when both physical endpoints are admitted.
  Connections to future cells are explicitly deferred boundary connections,
  **not successfully routed connections**. In the final run none may remain.
  This applies to routing-driven relocation too: both candidate input proofs
  and post-move input reconstruction must skip pending drivers. An unplaced
  driver in the current run remains an error; a future-run driver does not
  invalidate an otherwise legal relocation.
- Clock infrastructure required by admitted cells must be available in that
  run. Constants and generated local-chain helpers belong to their consumers'
  run. Infrastructure must not be recreated or erased between runs.
- A successful run has legal placement and complete routing for all currently
  eligible connections. Report timing as well; a timing allowance must never
  hide an unplaced cell or incomplete route.
- Freeze completed cells using their fixed flag. Later Outline, smearing,
  timing refinement, Sorting, Swapping and routing-driven movement must not
  change their coordinates or packing positions. They remain attraction anchors.
- Preserve completed routes and their numeric resource ownership. A new run
  must not reset the fabric or preempt completed work. New fanouts may extend an
  existing source tree without tearing it down.
- Placement operates over the existing occupied device. Actual Element packing
  compatibility and available capacity remain authoritative. Fully occupied
  candidates can be skipped cheaply; an abstract occupancy count must not
  replace the exact packing check for mixed cells and chains.
- On failure, report the run, stage, failed work and diagnostics, stop, and do
  not admit another part. A timeout means incomplete search, not mathematical
  proof of impossibility. Keep the failed state for diagnosis.

Every run must log its cell target, selected/new/completed/pending cells,
highest timing deficit, eligible and deferred connections, placement and routing
durations, stage/pass results, and final success or failure. Stage timeout
budgets restart per run; no successful earlier run may be silently redone.

## Development plan and verification

1. **Architecture and Estimate relocation.** Move the existing implementation
   without changing its formula; update build paths and run Estimate regressions.
2. **Run planning.** Implement deterministic criticality order, unique coverage,
   bunch/strict-chain integrity, size validation and explicit run accounting.
   Regression source: `src/tests/pnr/pnr.cpp`.
3. **Incremental engines.** Introduce active/pending membership; preserve fixed
   completed cells, routes, clocks, constants and source-tree ownership. Check
   real stage traversal, not merely a mock scheduler.
4. **Integration.** Expose the full orchestrated flow through Tcl, exercise
   multiple small runs with real placement and routing, and test deferred edges,
   capacity pressure, immutable earlier runs and failure stopping later runs.
5. **Scale validation.** Run existing regressions and example/puzzle workloads;
   report observed success and limitations separately. A passing planner test
   alone is not validation of incremental place and routing.

Implementation progress and measured validation are recorded below as each
level is completed; these requirements describe the target, not an assertion
that every level is already implemented.

## Invocation

After loading the device, RTL and constraints, use:

```tcl
open_design
pnr_design -run_cells 7500
```

`pnr_design` without options uses the CMake default
`-DSCALEPNR_PNR_RUN_CELLS=7500`. The command estimates automatically if no
Estimate forest exists. It returns the number of completed runs on success,
and a Tcl error on failure. The standalone `place_design` and `route_design`
commands remain available for stage diagnostics.

Start orchestration with a fresh routing state; restarting from an already
routed checkpoint is not implemented. Net storage is reserved before the first
run, so later runs cannot invalidate retained routing-owner pointers by growing
the net array. Each run uses a temporary view of the selected bunches and
restores the original Estimate forest afterward. Completed cells keep both
their physical assignments and their original bunch references.

The example script supports `SCALEPNR_PNR_RUN_CELLS=7500` as an explicit opt-in
to this flow. It rejects combining that mode with placement-cache, place-only
or clock-only modes. Large hardware examples have not yet established that
incremental execution improves routability; do not confuse synthetic regression
success with that result.

Run timing output is a placement-distance estimate, not routed sign-off.
`unplaced_edges` explicitly identifies timing paths crossing into future work;
their WNS is provisional. Routing completion is checked independently of WNS.

## Implemented and verified

Levels 1–4 above have an initial working implementation. The default target is
7500 cells, with explicit oversized groups. Clock infrastructure is grouped
with its highest-ranked consumer, rather than admitting unused buffers alone
or making every register in a clock domain one indivisible group.

Focused validation (24 tests passing):

- `src/tests/pnr/pnr.cpp`: criticality ordering, unique coverage, orphan cells,
  indivisible bunches/mux/carry chains, two-domain infrastructure admission,
  invalid sizes, deferred edges in both admission directions, retained fanouts,
  completed coordinates and route fragments, exhaustive lease/owner auditing,
  clock extension, constants, and stopping after a capacity failure.
  `pnr.moving_deferred` additionally forces source relocation with an unplaced
  future driver: the current output must complete without admitting that
  driver or queuing its input. The same unplaced driver, when admitted, must
  still reject relocation.
- Two 240-cell/20×20 synthetic designs, one chain and one branching tree:
  eight complete runs of 30 cells; all 239 data connections routed. Observed
  test runtimes were approximately 0.8 s and 1.7 s in the development build.
  These are correctness exercises, not 5–10K-cell performance evidence.
- `src/tests/pnr/pnr.tcl`: real Tcl registration, argument validation and
  unloaded-design rejection.
- Existing Estimate, packing, routing and 13 multi-clock regressions, including
  a new two-domain retained-clock-tree extension case.

Example 1 was exercised with a 5000-cell target (5000 + 1122 cells). It exposed
source relocation incorrectly proving future-run inputs. After fixing both
input reconstruction sites, Moving Sources improved from a 600-second timeout
with 17 trunks remaining to completion in 91.4 seconds. The full example still
failed in run 1: Fanouts reached its 600-second budget with 5715 tasks left,
and Moving Destinations exhausted a 13-candidate forward search. All 13
candidate congestion audits had zero unowned leases, missing leases, missing
registrations, or stale registrations. This is progress past one reproduced
bug, not proof of complete incremental routing or second-run hardware behavior.
The fixed run's reports are under `tests/prjxray/1/build/run.SBGlreTz/`.

Level 5 remains: measure complete hardware examples and 5–10K-cell runs before
making this the default example workflow. Checkpoint resume is not supported;
reload the design after a failed run. Bunches larger than the target remain
oversized rather than being split; partitioning such bunches is a separate
enhancement. The current implementation preserves the existing search
algorithms and does not guarantee that a failed search proves unroutability.

## Small-run scaling diagnostics

Use `SCALEPNR_PNR_RUN_CELLS=1000` (then smaller if necessary) together with
`SCALEPNR_ROUTE_PROFILE=1 SCALEPNR_ROUTE_PASS_DETAIL=1` on the example script.
Keep the same design, fabric and timing constraints. Count actual admitted cells:
the 1000-cell target selected 727 cells in its first run because bunches are
indivisible. Separate device import time from run/stage time. Do not increase
timeouts to disguise stalled search; reproduce a fault on a small case, add a
regression, then retry larger admission sizes.

The initial 1000-target baseline completed its first 727-cell run with Fanouts
in 11.95 seconds and no preemptions, versus the earlier 5000-cell Fanouts timeout.
Its next clock extension visited 8.35 million nodes for 79 sinks (39.86 seconds).
The following faults were reproduced and corrected:

- Run-size counters followed pending and completed upstream cells, inflating
  Outline and routing budgets from the full RTL cone. `pnr.run_budget` checks
  both pending and completed exclusions across real placement/routing runs.
- Retained clock trees disabled batch routing, restarting graph search for
  every new sink. Batch search now starts from all retained tree nodes with
  their original parent/visited-Tile histories. `pnr.clock_batch` extends one
  routed sink by 64 more: popped nodes drop from 2208 to 66, with earlier wires
  and route identity preserved. The graph contains 101 discovered nodes.
- Per-branch clock cleanup skipped the source pin; removing the final branch
  left the root leased and prevented clock repair. `pnr.clock_cleanup` checks
  survival of shared ownership, final-root release, and successful rebuilding.

These regressions establish individual fixes, not complete example success.
Candidate-order construction is timed separately as `candidate_order_ms` in
the routing hotpath report to distinguish it from leasing and docking.
Full-fabric snapshots at ordinary Basic/Fanouts handoffs require
`SCALEPNR_ROUTE_HANDOFF_DUMP=1`; compact task diagnostics remain enabled.
Failure/timeout snapshots keep their existing policy. These normal handoff
dumps were about 200 MB even with fewer than 1000 admitted cells, because
they serialize the whole fabric, not only the current run.
Moving retains the first 1 MiB of automatic detailed packing diagnostics;
subsequent normal probes stop formatting verbose masks. A single 989-cell run
previously produced a 1.34 GB temporary trace for one unsuccessful LUT search.
Candidate failure replay remains fully detailed; use
`SCALEPNR_ROUTE_PACKING_TRACE=1` for an explicitly requested unlimited live trace.
Five-second Moving progress messages report candidate counts, packing/support
time and trace bytes even when no routing pass has finished.

The bounded trace also exposed self-blocking Moving reservations: an incident
binding using an obsolete source could be treated as another input after the
trigger's input had already been reserved. Moving now applies recorded source
retargets to the saved Moving trigger and incident bindings before assembling
its tasks. Updating bindings alone did not fix the hardware case: the saved
trigger still used source helper 1558 while its binding used replacement 1559.
The `fpga.moving_duplicate_input` regression preserves a routed trunk, introduces
an empty old-source binding for the same sink port, and verifies relocation,
terminal ownership, and absence of obsolete requeued tasks. Other physical
inputs still need distinct terminal reservations. `fpga.moving_stale_trigger`
also keeps the saved trigger on the old source; without normalization its
forward search finds no anchors in the small synthetic fixture.

The 2000-cell-target probe admitted 1989 cells. Its second Basic pass spent
2487 ms of 2577 ms of generic search-loop time constructing candidate order.
Cold ordering assembled a full 4096-bit temporary mask for each single-bit
set/test, including repeated sources from every local/DST. Single-bit access
preserves the ordering and avoids those whole-mask operations. The dense cold
priority regression in `src/tests/fpga/angle_priority.cpp` covers 256 local
nodes, mapped and fallback sources, lanes, and all target directions. Its
measured ordering time fell from 651 ms to 38 ms in the development build;
this is an isolated operation benchmark, not an end-to-end speedup claim.

With both Moving source-identity fixes, `run.fdURaSbA` grounded the formerly
stalled `...$5327/I4` connection in two resource probes (previously 57,014
unsuccessful probes were recorded before a diagnostic stop). The entire
second run's Moving-destinations stage used 8.362 seconds before a different
failure, not a timeout. At `(15,31)/DST:261`, all eight forward sources were
busy; the attached resource had no acceptable packing/input combination for
`...$4058`. Its candidate audit reported zero unowned leases, missing leases,
missing registrations, or stale registrations. This is not a claim that the
whole example routes successfully.

The subsequent 1000-target rerun (`tests/prjxray/1/build/run.cTxXRyZ7/`)
completed its first 727-cell run in 21.503 seconds of routing, versus 64.479
seconds in `run.fUBHWL8d`. Basic retained exactly 953 searches / 2380 pops
and fell from 9.603 to 4.059 seconds; Fanouts retained exactly 2626 searches /
12228 pops and fell from 15.918 to 5.956 seconds. Final estimated WNS stayed
2.015 ns. Thus this measured improvement did not come from skipping searches.

The 1989-cell probe (`run.ccYu5FtK`) was stopped diagnostically after 105
Fanouts passes, not declared successful or timed out. Its queue fell from
2461 to 236, with 3399 completions and 1638 completed-route preemptions.
Repeated docking/preemption remained a scaling issue; the low-level fixes do
not establish complete hardware routability.

The updated 2000-target probe (`run.xqkriPRy`) was deliberately stopped at the
same 105-Fanouts-pass checkpoint. The router's elapsed counter dropped from
495.967 to 363.315 seconds, with identical 3399 completions, 1638 completed
victims and 236 remaining tasks. Fanouts profiling attributed 252.091 seconds
to Docking, 6.428 seconds to the generic search loop, and 2.209 seconds within
that loop to candidate ordering. After Fanouts pass 20, 1049 completions were
offset by 949 completed-route preemptions. The remaining scaling problem is
therefore repeated Docking/preemption work, not the repaired Moving
self-reservation or cold ordering cost. This checkpoint was neither a full
example success nor a timeout. The focused routing/PnR regression suite
passed all 23 tests after these fixes.

## Detailed stage references

- [Placement stages and algorithms](../doc/placing.md)
- [Routing stages and algorithms](../doc/routing.md)
