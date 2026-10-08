# Outline strategy experiments — 2026-10-08

Baseline revision: `e5e2aa6`. No production change was retained: the tested
Outline improvements either regressed WNS elsewhere or increased downstream
work enough that the small improvement was not compelling.

## Method

The main comparison uses the 100×100, 50% puzzle, 160,000 core cells and its
existing 1 ns clock. All variants retain the same generator, fixed I/Os,
timing calibration, 100 bunch iterations, 478 instance iterations and movement
limit. No test acceptance threshold was changed.

Outline-only measurement uses the existing `SCALEPNR_PLACE_OUTLINE_AUDIT=1`
and `SCALEPNR_PLACE_OUTLINE_AUDIT_SKIP_TRIALS=1` switches with
`build/placing_puzzle_100_test 100 50`. Smaller comparisons use that same
optimized executable with `50 50`. Some runs overlap, so elapsed times are
not suitable for claiming a speedup. Outline's quality comparison uses fixed
iteration counts, not timeout-dependent stopping.

## Measurements

All timing figures are in ns. Higher WNS and lower TNS are better.

| Change | 50×50 Outline WNS | 100×100 Outline WNS | 100×100 Outline TNS |
|---|---:|---:|---:|
| Baseline | +0.165 | −0.579 | 12.631 |
| End density spreading after 50 instead of 100 iterations | +0.200 | −0.571 | 12.169 |
| Average register input/output forces separately | +0.225 | −0.624 | 8.922 |
| Combine those two changes | +0.225 | −0.624 | 8.399 |
| Stronger local COMB correction between register roots | +0.165 | −0.579 | 10.734 |
| Additional pressure on long register connections | +0.200 | −0.579 | 11.503 |
| Double force-only relaxation gain | +0.015 | −0.617 | 24.278 |
| Quadruple force-only relaxation gain | −0.135 | −1.580 | 88.949 |
| Resource-derived density target | +0.125 | −0.544 | 11.787 |
| Resource-derived target plus early density stopping | not separately sampled | −0.544 | 11.858 |
| Half resource-derived target, leaving packing headroom | +0.165 | −0.571 | 12.168 |

Combining register-side balancing with long-connection pressure was also
tested on 50×50: +0.225 ns, no further improvement there.

The changes were deliberately small:

- Density stopping retains the original total iteration budget; the remaining
  iterations perform only the existing timing attraction.
- Register balancing averages the weighted vectors within each signal side,
  then averages the sides, rather than averaging all peers together.
- Stronger COMB following replaces the 0.875 longitudinal root-force retention
  with zero only for two-sided remote-root followers without fixed-root influence.
- Long-connection pressure multiplies the existing register connection weight
  by `max(1, estimated_wire_delay / (0.5 * clock_period))`. It uses the existing
  horizontal/vertical calibration and converts the doubled planning grid back
  into physical-Tile distances.
- Faster relaxation multiplies the 0.10 attraction step after the density phase,
  retaining the original 0.75 planning-grid-unit displacement cap.
- Resource-derived density divides total LUT/register capacity by planning-grid
  area, with the original two-cell target as fallback/minimum. The synthetic
  32-slot Tile has four planning bins, giving eight cells per bin. The headroom
  variant uses four. These are coarse heuristics, not exact packing checks.

## Downstream validation

These full 50×50 runs all pass the current −0.300 ns test tolerance. Passing
does not establish an improvement over the baseline:

| Variant | Final WNS | Final TNS | Swapping attempts |
|---|---:|---:|---:|
| Baseline | −0.099 | 3.602 | 527,880 |
| Early density stopping | −0.179 | 5.548 | 663,054 |
| Balanced register sides | −0.099 | 3.443 | 663,289 |
| Resource-derived density | −0.192 | 6.663 | 771,630 |
| Resource-derived density plus early stopping | −0.119 | 5.285 | 741,104 |
| Half-capacity density / headroom | −0.099 | 4.333 | 689,066 |

Thus the strongest Outline-only improvement, 35 ps, worsened the full 50×50
result. The headroom variant preserved final WNS but gained only 8 ps on
100×100 Outline while increasing 50×50 swap attempts by 30.5%.
No full 100×100 flow improvement is claimed; the variants were screened out
before promoting them to that longer comparison.

Directional and large branched Outline checks passed for early density stopping
and balanced register attraction. The full-capacity variant also passed the
large branched check. Its existing minimum correlation (~0.60533), span
(0.811109) and WNS (−6.855 ns) remained effectively unchanged.
The headroom variant passed the full 12×12 single-clock and two-clock puzzles.

Focused experimental regressions confirmed register-side bias, faster bounded
relaxation and the density threshold's behavior. These tests were removed with
the rejected policies; the existing test expectations were not weakened.

## What the worst path proves

The original 100×100 Outline WNS belongs to:

`mesh_cell_108137 (57,62) → mesh_cell_108149 (54,59) → mesh_cell_108162 (44,53) → mesh_cell_109764 (35,48)`

The endpoint cells are registers; the intermediate cells are LUTs. The edge
Manhattan distances are 6, 16 and 14 Tiles, totaling 36. The two registers are
also exactly 36 Manhattan Tiles apart: **this path is already geometrically
taut**. Its wire delays are 0.242, 0.607 and 0.530 ns; intrinsic delay adds
0.200 ns, giving arrival 1.579 ns and slack −0.579 ns.

Moving only these LUTs cannot reduce the Manhattan component while the
registers remain fixed. At most the small bend penalties can be removed.
This explains why stronger COMB straightening improved TNS but did not repair
the worst endpoint. Faster attraction also does not guarantee a better
equilibrium, as the measured 2×/4× regressions show.

## Artifacts and rollback

Detailed logs are under `build/outline_strategies.Lnp3Jg/`; the original
100×100 baseline is `build/outline_baseline_20261008.log`. The directory's
`REPORT.md` maps temporary trial numbers to log names and records red/green
checks. These build artifacts are not tracked and may be removed by build cleanup.

All temporary controls and runtime/test edits were reverted to `e5e2aa6`.
Only this experiment report is retained.

After rebuilding the restored code, all 30 selected Outline, multi-clock and
PnR regressions pass (`ctest_restored.log`, 124.52 seconds). Full 12×12
single-clock, 12×12 two-clock and 50×50 puzzles also pass. The restored 50×50
run reproduces baseline final WNS −0.099 ns, TNS 3.602 ns and 527,880 swapping
attempts exactly. `git diff -- src` is empty.
