# Outline attraction from current setup slack — 2026-10-08

Baseline: `e5e2aa6`. These experiments are separate from the earlier static
attraction/density trials in `outline_experiments_20261008.md`.

## Method

Refresh the existing attraction weights every 25 instance iterations, using
the complete critical path's current Outline-coordinate setup slack. Do not
change the generator, density passes, iteration budget, movement cap, timing
calibration, packing rules, or test thresholds.

For normalized slack `s = slack_ns / max(required_ns, 0.05)` and strength `k`,
multiply the original connection weight by:

- `1 / (1 + k*s)` when `s >= 0`: attenuate paths with timing headroom.
- `min(4, 1 - k*s)` otherwise: increase pressure on violated paths.

Shared connections use their most critical endpoint's factor. Every refresh
starts from the original guide, not the previously multiplied weight.
The timing forest is evaluated again so a previously noncritical input can
become critical. Registers remain attraction sources; COMBs retain faded
following. No placement or packing state is changed by guide evaluation.

## Initial coefficient sweep

Times below are not speed benchmarks: some trials overlapped compilation or
other tests. Outline has a fixed iteration count. Full-flow repair stages are
time-budgeted, so their results must not be treated as fixed-work comparisons.

| Strength | 100×100 Outline WNS | Outline TNS | 50×50 Outline WNS | 50×50 final WNS | Final TNS | Swap attempts |
|---|---:|---:|---:|---:|---:|---:|
| Original guide | −0.579 | 12.631 | +0.165 | −0.099 | 3.602 | 527,880 |
| 0.25 | −0.571 | 9.672 | +0.200 | −0.099 | 2.944 | 436,588 |
| 2 | −0.342 | 2.862 | +0.235 | −0.082 | 1.295 | 278,072 |
| 8 | −0.717 | 5.528 | +0.021 | −0.119 | 2.901 | 350,529 |

The 50×50 runs finish without repair timeouts. A zero-strength control produced
identical 12×12 audit paths and histories to the unmodified guide.

The original worst 100×100 Outline endpoint, `mesh_cell_109764`, improves from
−0.579 to −0.504 ns even with strength 0.25; another path becomes worst.
With strength 2 the worst endpoint becomes `mesh_cell_158588`, at −0.342 ns.

## Full 100×100 check at strength 2

| Stage | Original | Strength 2 |
|---|---:|---:|
| After PlaceTiming | −4.214 | −3.759 |
| After Sorting | −3.557 | −1.944 |
| After Swapping | −1.259 | −0.629 |

Both final runs fail the existing puzzle tolerance. Sorting reaches its
60-second limit in both; Swapping uses its remaining budget/recovery reserve.
Compilation overlap and different numbers of repair moves mean this table is
an observed full-flow comparison, not proof of a performance speedup.

## Regression gate

Strength 2 passes the timing regression and most focused regressions but fails
the two anchored Outline geometry cases. Directional interior span falls to
0.78524 (minimum 0.80); the large branching test has minimum correlation
0.59504 (minimum 0.60) and minimum span 0.75523. These thresholds are unchanged.
This setting is therefore not accepted solely on its puzzle timing improvement.

Preserving at least the original fixed-I/O attraction at strength 2 still
fails both geometry cases. That experimental exception was removed.

## Initial selection: strength 0.25 (superseded below)

The mild setting passes both geometry cases without changing their thresholds:

- Directional: minimum correlation 0.970343, minimum interior span 0.816271.
- Large branching: minimum correlation 0.603949, minimum interior span 0.800428;
  estimated WNS improves from the original −6.855 to −6.215 ns. Its span is
  closer to the test's 0.80 lower bound than the baseline's 0.811109.

The initial decision was to keep strength 0.25, refreshing every 25 iterations,
and remove all experimental
environment switches and the fixed-I/O exception. The stronger mesh-only gain
does not justify failing the existing anchored-chain geometry regressions.

The new timing regression checks exact Outline/physical timing agreement on
a non-square chip, full COMB-path delay, two clock requirements including an
explicit setup limit, fixed physical coordinates, shared-path maximum pressure,
symmetry, zero-strength identity, non-compounding refreshes, switching of the
critical input, and no mutation of placement/packing.

Final retained-code verification: **32/32 focused timing, Outline, multi-clock
and incremental-PnR cases pass** (132.62 seconds). Full 12×12 single-clock,
12×12 two-clock and 50×50 puzzles pass. The final 50×50 run reproduces WNS
−0.099 ns, TNS 2.944 ns and 436,588 swap attempts exactly.

For the original worst chain, mild feedback changes the Outline targets to
`(56,62) → (54,59) → (44,54) → (35,49)`. Its total Manhattan length drops
from 36 to 34 Tiles, and its slack improves from −0.579 to −0.504 ns. This
is a small register-span correction, not a complete solution of that path.

The retained full 100×100 run ends at **−0.932 ns WNS**, still outside the
unchanged puzzle tolerance. Its stage results are:

| Stage | Original run WNS | Retained mild WNS |
|---|---:|---:|
| Outline | −0.579 | −0.571 |
| After PlaceTiming | −4.214 | −4.129 |
| After Sorting | −3.557 | −3.932 |
| After Swapping | −1.259 | −0.932 |

Final TNS is 1898.92 ns versus 2533.6 ns in the original run. Sorting reaches
its 60-second limit; Swapping completes five passes (four improving), retains
394 swaps and uses its recovery reserve. The same time-budget caveat applies:
these downstream runs are not fixed-work comparisons and not every stage wins.

The 19 guide refreshes account for 18.555 seconds of the retained run's
313.286-second Outline stage (~5.9% of that measured stage). This is measured
refresh cost, not a controlled whole-stage slowdown estimate.

## Artifacts

Detailed logs and the standalone regression-link helper are in the ignored
directory `build/outline_slack.SQr6AR/`. The helper recompiles target-specific
test sources and links them against the updated shared debug engine objects,
checking matching compiler definitions/options. It preserves target-specific
link libraries; it does not alter the repository's CMake target definitions.

## User-approved geometry tradeoff: strength 2

The user subsequently approved relaxing the heuristic geometry limits to
retain the larger timing improvement. The default strength is now **2.0**.

| Quality check | Previous minimum | New minimum | Measured strength-2 result |
|---|---:|---:|---:|
| Directional interior span | 0.80 | 0.75 | 0.78524 |
| Branching interior span | 0.80 | 0.75 | 0.75523 |
| Minimum path correlation | 0.60 | 0.59 | 0.59504 |

These are explicit quality-policy changes, not a correction to timing math or
an assertion that the old geometry scores improved. Aggregate ordering checks,
fixed-I/O immobility, complete graph coverage, generation checks, and the
full placing puzzle's timing tolerance remain unchanged. The geometry-check
self-test now also verifies that 76% interior span is permitted while 70%
span and complete collapse remain detectable.

The approved-policy rerun passes **32/32 focused regressions** (124.98 seconds),
including both previously rejected geometry cases. Full 12×12 single-clock,
12×12 two-clock and 50×50 puzzles pass. The 50×50 run reproduces Outline
WNS +0.235 ns, final WNS −0.082 ns, final TNS 1.295 ns and 278,072 swap attempts.

The full 100×100 rerun reproduces Outline WNS **−0.342 ns**, TNS **2.862 ns**.
After PlaceTiming WNS is −3.759 ns; after Sorting −2.429 ns; after Swapping
**−0.832 ns**, with final TNS 1321.6 ns. It still fails the unchanged full-flow
timing tolerance. Sorting reaches its 60-second budget; Swapping executes six
passes and retains 506 swaps. Its different result from the earlier −0.629 ns
run reflects the time-budgeted downstream search, not a different Outline WNS.
