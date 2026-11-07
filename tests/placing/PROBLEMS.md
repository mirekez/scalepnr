# Placement problems found and fixed

The generated RTL, focused regressions and per-stage metrics retain the evidence
for these fixes. Final measurements are recorded in `validation.json`.

## Whole-design Outline was inside each incremental run

Inspection of `Pnr::run` and `Tech::placeDesign` found that pending flags and a
stage-local bunch view were installed before Outline. Thus Outline saw only the
admitted chunk rather than the complete design. Move Outline before planning and
pending admission, and skip repeat outlining in the later placement calls.

## Shared logic could be skipped by Outline (1k stencil and FIFO)

Initializing all imported core coordinates to `(-1,-1)` exposed cells that
retained that sentinel after Outline. Estimate may transfer a shared COMB to a
deeper cone; its ownership region need not remain connected to its bunch root.
Same-bunch-only allocation/preparation skipped such cells. Simply crossing the
boundary could also enter a register with a null bunch and mark it visited before
its explicit tree traversal, hiding its entire subtree.

Allocation and graph preparation now cross non-root ownership boundaries and
reserve bunch-root entry for the explicit tree. Density processing also sweeps
unvisited prepared graph members. The 1k stencil and FIFO reproduce these cases;
all ten 1k designs pass with sentinel initialization and full coverage checks.

## Estimate overstated transferred combinational demand (1k stencil)

The 931-cell stencil reported 5,589 combinational cells in its aggregate demand.
Recursive discovery added transferred cells to the new owner without removing
old counts. Recount final ownership once after discovery/aggregation and rebuild
subtree totals before Outline. The harness independently compares total estimated
COMB demand with the number of imported LUT cells.

## Placement cleanup could return with missing cells

The existing cleanup loop diagnosed no progress and broke without a final
completeness check. It now throws if an admitted packable cell remains unplaced.
Every synthesized test also independently checks completeness after each stage.

## `cmake --build` had no make command

The top-level build set `CMAKE_MAKE_PROGRAM` as a non-cache variable. Configuration
succeeded but `cmake --build` failed with an empty make command. Cache the existing
selection; the normal build command now works.

## Native stack overflow in deep Estimate traversal (30k stencil)

The unmodified 30,061-cell stencil exited with SIGSEGV before Outline. GDB
located the failure in `EstimateDesign::recurseComb` at DFS depth 16,484 with
an 8 MiB process stack. This is a long traversal across many registers, not an
unrealistically long register-to-register combinational path.

Estimate now suspends its existing register/COMB DFS calculations in heap-backed
coroutine frames driven by an explicit work stack. Visit order, ownership
transfers and post-order timing calculations are retained; no stack-limit or
clock-target workaround is used. `placing.estimate_design.deep` independently
checks a 24,000-cell cone's ownership, exact cell count and accumulated delay.

## Timing ignored connections to future chunks (30k stencil)

After the stack repair, the unchanged 30k stencil placed legally but finished
with WNS −2.536 ns against its 6 ns period. Wires involving pending cells had
zero estimated delay, allowing early decisions to look safe before freezing.
Use whole-design Outline coordinates for pending cells in full, prepared and
local timing evaluation; preserve the separate count of physically unplaced
edges. Swapping's geometric projection uses the same coordinates. The focused
timing regression checks a known projected delay, evaluator agreement and
identical timing after admission at the projected location.

## Shared COMB calculations split across frozen runs (30k stencil)

Pending-wire estimates alone improved the miss to −1.131 ns but did not close it.
The critical path crossed the chip repeatedly through COMBs owned by different
capture bunches. The planner now keeps connected COMB networks and their owning
bunches in the same run, stopping at launching registers/buffers. This retains the
1000-cell target and permits oversized indivisible groups as before. A planner
regression checks both shared-cone integrity and the register boundary. This
reduced the remaining miss to −0.411 ns; it was not sufficient alone.

## Outline's coarse capacity phase was unreachable (30k stencil)

The capacity-spreading branch required `i > 100`, but the bunch iteration budget
is capped at 100. Thus no size ever executed it. Schedule capacity spreading in
the latter half of the existing budget, after initial attraction. No iteration
limit, clock period or slack tolerance is increased. Outline reports its actual
capacity iterations; the synthesized tests require this phase to execute.
The unchanged 30k stencil then passed all 31 placement chunks and the −0.10 ns
acceptance threshold. The 25 focused placement/PnR regressions also passed.

## Fixture overstated general LUT capacity

The abstract device has separate inverter-only and general LUT resources.
Counting both in Outline's scalar general-LUT budget doubled the usable LUT2–4
capacity. Report only general LUT sites in that budget; keep the independent
physical masks for inverter slots and check both classes for collisions. This
correction initially had no effect, which exposed the unreachable capacity phase
above. Outline peak demand and excess demand are retained as diagnostics;
smearing remains responsible for exact resource assignment.

## Shared arithmetic still scattered inside one run (50k stencil)

Nine 50k families passed, but the stencil finished at −1.714 ns. Keeping its
connected COMBs in one run prevented early freezing through a calculation, yet
pre-smearing still reserved each bit-slice capture bunch separately. Thus one
arithmetic cone could occupy distant parts of the fabric within that run.

Coalesce connected COMB owners in the existing temporary run-bunch view. Packing
now reserves a coherent footprint and the later optimizers see the same group.
Fixed I/O anchors remain separate; the original Estimate ownership and tree are
restored after the run. No clock, capacity or search-limit relaxation is used.
The unchanged 49,993-cell stencil then completed 51 chunks at −0.032 ns. The
`pnr.shared_cone` regression checks temporary grouping, all three later stages,
physical placement and restoration of the original distinct owners.

## Shared outputs left capture registers behind (80k arithmetic)

The 80k arithmetic pipeline exposed a two-edge detour: launch and capture were
both at (0,26), while their shared LUT was at (60,15). The LUT fed two capture
registers but only one belonged to its original owner. Final slack was −0.214 ns.

Admit additional capture registers with a shared calculation when their combined
group fits the existing run target; include those captures in the temporary
packing view too. Register outputs remain launch boundaries. The explicit size
guard prevents a shared enable/reset from combining a whole clock domain into
one oversized run. The unchanged 80k arithmetic pipeline now finishes at
+1.591 ns. Planner and placement regressions cover multiple captures, the size
guard and restoration of distinct original owners.

## Frozen edges could exhaust Swapping's critical-path budget

Investigation of the 80k stencil miss exposed that Swapping applied its edge
budget before checking mobility. Long frozen wires could displace shorter,
repairable edges. A focused fixture reproduces this independent defect. The
stencil itself had an additional tied-path problem, so filtering the search
list alone did not repair it.

Filter edges with missing groups or two fixed groups before applying the
existing critical-edge limit. Keep the same search budget, clock period and
−0.10 ns acceptance threshold. The focused Swapping regression adds a longer,
fixed prefix to a repairable path and uses a one-edge budget; it verifies an
accepted repair and unchanged fixed coordinates.

## Sorting rejected progress through tied critical paths (80k stencil)

After the earlier fixes the 80k stencil still missed by 2.4 ps beyond the allowed
−0.10 ns slack. Its calculation/capture cone was placed in chunk 28; the launch
registers arrived in chunk 69. Legal neighborhood trials showed that shortening
one launching wire left endpoint WNS unchanged because another equally critical
path remained. Sorting required strict endpoint-slack improvement and therefore
rejected each repair. Swapping had no suitable exchange partner.

Sorting now recognizes progress through such ties: unchanged setup slack is
acceptable only when the number of equally critical paths strictly decreases.
The existing affected-path guard remains, and a neutral repair cannot increase
the critical-path count at an equally bad or worse endpoint. Counts follow all
branches of the shared timing forest, with memoization within each prediction.
The focused regression has two equal-delay launch paths converging at fixed
logic; it requires a neutral repair followed by closure, preserving fixed cells.
It fails against the original Sorting implementation (zero accepted moves at
−0.135 ns) and passes against the repair.
No clock period, timing tolerance or iteration budget is relaxed.
The unchanged 80,025-cell stencil completes 82 runs at WNS -0.097429 ns,
with legal placement and no remaining unplaced timing edges.

## Swapping opened a local repair after Sorting had finished (100k stencil)

The 100,019-cell stencil met timing through chunk 100. In chunk 101, Sorting
improved WNS from −0.719 to −0.679 ns; three accepted swaps then removed 17 of
18 actionable endpoint violations, leaving −0.136599 ns. The remaining logic
and capture register were newly placed; their launch register was frozen in
chunk 30. The exchanged bunch's new geometry permitted a local repair that
Sorting could not make before the exchange.

A checkpoint replay reproduced WNS/TNS exactly. The existing local timing
refiner made no improvement; rerunning the ordinary Sorting stage improved WNS
to −0.097429 ns and TNS from 2.16429 to 2.08429 ns, preserving fixed cells.
After accepted swaps that leave an actionable deficit, invoke Sorting once more
before freezing the run. This reuses its existing capacity checks and affected-
path timing guard. There is no new placement algorithm, clock relaxation or
increased iteration setting. The optional pass has its own audited stage and
metrics; the log parser verifies the extra Sorting summary.
The complete 100,019-cell CTest now passes all 102 runs at final WNS
−0.097429 ns with zero unplaced timing edges. Exactly one post-swap Sorting
pass executes, in run 101; its WNS/TNS changes match the checkpoint replay.

## FIR workload pipeline alignment

RTL review found that the third FIR product bypassed the registered partial sum,
combining two intended taps in the same sample interval. Delay that product by
one register before the final sum, then recalibrate and rerun this family in
each completed tier. An eight-step Yosys SAT impulse trace of one section gives
the intended consecutive coefficients 3, 5, 7. This corrects the workload rather
than using spare cells as padding.

## Focused regression validation

After the final placement change, 29 focused CTests pass: Estimate (including
the 24k-deep traversal), Outline and its 100k fixture, timing evaluation,
Sorting, Swapping, the placement puzzle, incremental PnR planning/grouping,
routing-pressure integration, clock/constant handling and the Tcl entry point.
These supplement the synthesized workload results in [RESULTS.md](RESULTS.md).
The final complete workload rerun also passes all 71 CTests (70 designs plus
Yosys setup), covering 3,019 placement runs with one common placer binary.
The measured Sorting and Swapping stages never regress WNS. Only run 101 of
the 100k stencil requires the conditional final Sorting pass.
