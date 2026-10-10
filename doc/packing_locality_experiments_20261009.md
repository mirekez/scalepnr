# Packing locality investigation — 2026-10-09

## Reproduced loss

The current 100x100 two-clock puzzle finishes Outline at WNS -0.307 ns,
TNS 2.352 ns. Exact packing, before internal timing refinement, changes those
to -2.759 ns and 720.346 ns. All 160,000 core cells are committed by
pre-smearing reservations; later unreserved-cell smearing makes zero moves.

The worst packed chain is:

| Cell | Outline | Packed | Members in its reserved bunch | Reservation rank (zero-based) |
|---|---|---|---:|---:|
| mesh_cell_111165 | (47,78) | (57,97) | 2 | 102350 |
| mesh_cell_112765 | (47,78) | (21,84) | 2 | 102295 |
| mesh_cell_114357 | (47,80) | (60,96) | 1 | 103973 |

The two edges grow from 0/2 to 49/51 Manhattan Tiles. The middle cell's
two-cell footprint occupies (19,84)..(21,84); the source's two-cell footprint
occupies (57,97)..(60,97). This is not a large bunch that cannot fit one Tile.
Connected small footprints independently escape to different sides of the
crowded region. The old radial origin remains the original Outline home;
candidate scoring cannot recover alternatives outside its narrow search band.

## Controlled experiments

Trials fork after one shared Outline execution. Each child starts with the same
cells, addresses, coordinates, timing graph, and fixed occupancy. It calls real
pre-smearing and exact reservation commit, then checks legal final coordinates,
complete cell placement, endpoint count and zero unplaced timing edges. No
generated coordinates guide the algorithm; they are used only for reporting.
Repeated baseline trials reproduce WNS and TNS exactly.

No clock period, timing calibration, packing legality, test acceptance threshold,
or bunch's exact internal shape is relaxed.

| Policy | 100x100 two-clock packed WNS | TNS | 50x50 packed WNS | TNS |
|---|---:|---:|---:|---:|
| Baseline | -2.759 | 720.346 | -0.414 | 2.876 |
| Follow half of already-reserved neighbours' displacement | -2.809 | 817.735 | not run | not run |
| Follow all of already-reserved neighbours' displacement | -2.719 | 822.437 | not run | not run |
| Largest member count first | -2.314 | 19460.4 | -0.384 | 62.887 |
| Largest footprint area first | -3.092 | 20600 | -0.474 | 83.040 |
| Largest count first + half neighbour displacement | -2.309 | 19196.1 | -0.414 | 57.387 |
| Largest count first + full neighbour displacement | -2.334 | 17974 | -0.414 | 46.731 |
| Largest area first + full neighbour displacement | -2.514 | 20327.7 | -0.494 | 82.864 |
| Half last local displacement | -3.174 | 533.400 | -0.544 | 1.267 |
| Last local displacement | -2.074 | 363.552 | -0.684 | 9.665 |
| Mean local displacement | -2.654 | 424.799 | -0.327 | 1.028 |
| Equal last/mean blend, no future projection (first candidate) | -1.989 | 269.172 | -0.324 | 3.087 |
| Last local displacement + projected future peers | -3.999 | 2226.15 | -1.724 | 91.660 |
| Mean local displacement + projected future peers | -3.427 | 1061.26 | -1.094 | 14.043 |
| **Equal last/mean blend + projected future peers (selected)** | **-1.894** | **209.514** | **-0.689** | **4.039** |

Local displacement means accepted translations belonging to one original
Outline Tile. Immediate cardinal neighbours are consulted only when that Tile
has no samples. It does not average a connected component or the whole chip.
Hints smaller than two Manhattan Tiles leave the original search centre
unchanged, preserving small home/adjacent-Tile decisions. Existing fixed cells
do not move. Reordering and undamped future projection are rejected: their
small WNS gains or isolated improvements hide severe TNS regressions.

The displaced-neighbour regression in `src/tests/placing/place_timing.cpp`
reproduces a free but obsolete sink home: its source bunch has to move six Tiles
because nearby LOGIC slots are occupied, while its sink's original REG slot is
free. The baseline tears that connection apart. Following local displacement
passes in both axes, preserving the source bunch's offset, existing occupants,
and the setup requirement.

Raw trial logs are `/tmp/scalepnr-pack-{rebase,order,field,project,blend}-sweep*.log`.
Packing-only results are not claims about final Sorting/Swapping WNS or a passing
full suite. Full-flow validation and the retained policy are recorded below when
complete.

## First candidate

Use the equal last/mean local displacement blend solely to update the radial
search origin. Keep the original linear bunch order and score future peers at
their real Outline positions. All experimental switches and fork-sweep code
were removed after selection. The existing narrow radial band, cubic timing
cost, fixed-cell protection and precise packing preview remain unchanged.

On the controlled 100x100 replay, maximum bunch translation falls from 33 to
26 Tiles; reservation/commit/analysis time changes from 3.675 to 2.822 seconds.
These are observed trial times, not a general performance guarantee. WNS loss
is reduced, not eliminated: -1.989 ns still follows an Outline WNS of -0.307 ns.
50x50 packed TNS increases slightly, from 2.876 to 3.087 ns, despite better WNS;
the full-flow checks must determine whether the later stages repair it.

## First full-flow validation (blend without peer projection)

The full rebuild and all 81 non-puzzle regressions passed. The 100x100 two-clock
flow reproduced packed WNS -1.989 ns / TNS 269.172 ns. Internal refinement
improved WNS to -1.204 ns (previously it stayed at -2.759 ns). External
PlaceTiming reduced violations to 645, versus 2469 before this change. Sorting
took 6.621 seconds and Swapping took 95.871 seconds in this run.

The puzzle nevertheless FAILS: final WNS -0.499 ns, TNS 44.562 ns, four
actionable endpoints remain. Swapping finished after 14 passes with no accepted
improvement in its last pass; this was not a timeout. Total test time was
339.72 seconds. No threshold was changed. The worst final edge,
mesh_cell_112976 -> mesh_cell_114581, is internal to one bunch; its distance
was 9 after packing but is 33 at the end. The existing compact-COMB fallback
is enabled for internal edges, so the same-bunch skip in the rigid phase alone
does not explain its remaining violation.

Full-flow raw logs: `/tmp/scalepnr-pack-final-regressions.log`,
`/tmp/scalepnr-pack-final-2clocks.log`. The damped peer-projection candidate was
then evaluated separately, as recorded below.

## Selected policy: damped local hints for both origin and future-peer score

Keep the original linear bunch order. Use the same equal last/mean local hint
for the group's radial origin and for estimating where unreserved movable peers
will be placed. Fixed peers are never projected; already reserved peers use
their actual positions. No peer coordinates are changed by the estimate. The
bunch lookup is reindexed after sorting so its original-home lookup is correct.
Undamped projection variants remain rejected. Experimental switches were removed.

A second focused case makes the scored peer still unreserved. Scoring its
obsolete position puts a two-Tile gap across an occupied seed Tile; using the
local projection keeps the final pair one Tile apart and meets setup timing.
It fails with projection disabled and passes with it enabled, in both axes.

The selected full 100x100 two-clock run PASSES its unchanged -0.400 ns limit:

| Stage | Before this packing work | Selected |
|---|---:|---:|
| Outline | -0.307 | -0.307 |
| Exact packing | -2.759 | -1.894 |
| Internal refinement | -2.759 | -1.321 |
| PlaceTiming | -2.759 | -1.321 |
| Sorting | -2.654 | -0.789 |
| Swapping | -0.433 | -0.391 |

Final TNS is 29.015 ns (previously 137.285 ns), with four swapping passes and
280.32 seconds total runtime. The final per-clock WNS values are -0.304 ns and
-0.391 ns. This still contains negative slack; a passing puzzle uses the existing
approved tolerance, not zero-slack closure. This first run used the experimental
enable flag. Raw log: `/tmp/scalepnr-pack-project-flow-2clocks.log`.

## Default-build validation

After removing the experimental switch and rebuilding all targets, all 81
non-puzzle regressions pass (482.99 seconds), including both new packing cases.
All five placing puzzles also pass, without timeouts or threshold changes:

| Puzzle | Final WNS (ns) | Total time (s) |
|---|---:|---:|
| 12x12 single-clock | -0.119 | 1.64 |
| 50x50 single-clock | -0.082 | 87.12 |
| 100x100 single-clock | -0.361 | 269.65 |
| 100x100 two-clock | -0.391 | 278.24 |
| 12x12 two-clock | -0.119 | 0.40 |

The default two-clock run exactly reproduces the selected experiment's packed
and final timing values. Its final TNS is 29.015 ns; four swapping passes take
37.565 seconds. The single-clock 100x100 run has packed WNS -1.829 ns,
post-internal-refinement WNS -1.161 ns, post-Sorting WNS -0.669 ns, and final
TNS 23.072 ns. The 50x50 packed regression (-0.689 ns) is repaired to -0.082 ns
by one swapping pass; it is not hidden by changing a limit.

Raw logs and JUnit reports:
`/tmp/scalepnr-pack-selected-regressions.{log,xml}` and
`/tmp/scalepnr-pack-selected-placing.{log,xml}`.
All six routing puzzles also pass, run separately and sequentially:

| Routing puzzle | Total time (s) |
|---|---:|
| 10K basic | 42.35 |
| 10K with fanouts | 47.77 |
| 10K, 90% fullness | 48.90 |
| 50K, 50% fullness | 251.92 |
| 50K, 90% fullness | 267.21 |
| 50K fanout forest | 242.80 |

The forest completes all 22,000 Basic tasks and all 28,000 suffixes. All routing
puzzles remain within their original 300-second per-test limits. Raw log and
JUnit report: `/tmp/scalepnr-pack-selected-routing.{log,xml}`.

Final result: **92/92 registered tests pass (81 regressions + 11 puzzles)**.
No timeout or acceptance limit was changed by this packing work. The existing
100x100 limit is -0.400 ns; passing these puzzles does not claim zero-slack
closure. The full build and `git diff --check` also pass.
