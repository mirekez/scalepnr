# Pre-smearing and exact packing experiments — 2026-10-08

Baseline committed first: `3771132` (Outline setup-slack feedback, strength 2).
No packing rules, timing calibration, puzzle generation, or test thresholds
were relaxed for these experiments.

## Where the loss happens

In the 100×100, 50% puzzle, Outline gives WNS −0.342 ns and TNS 2.862 ns.
Pre-smearing reserves all 160,000 core cells in 106,713 bunches, and exact
packing commits all those reservations. There are no remaining core cells
for the later unreserved-cell smearing step. The resulting packed WNS is
−3.759 ns and TNS is 2917.478 ns, before internal timing refinement.

For example, `mesh_cell_110930 → mesh_cell_112541` changes from
`(45,73) → (45,74)` in Outline to `(39,98) → (16,75)` in exact packing:
Manhattan distance 1 becomes 46. Some spreading is necessary: 1,774 Outline
target Tiles exceed their 32-cell capacity. The problem is how reservations
distribute that necessary displacement, not the mere fact that cells move.

## Controlled trials

Each comparison forked after Outline, so every child started from identical
cells, coordinates, timing graph and resource occupancy. Each child performed
pre-smearing, exact packing and internal refinement, checked complete legal
placement and endpoint coverage, and reported the pre-refinement timing.
All repeated baselines reproduced the same WNS and TNS.

The temporary experiment switches and fork harness were removed after
selection. Raw local logs are in `build/prepack_experiments.u4lCMw/`:
`sweep_{50,100}.log`, `fine_{50,100}.log`, `cost_{50,100}.log`.
Times from these overlapping runs are observations, not isolated benchmarks.

Here, “band N” compares N extra radial rings after the first cheap-capacity
candidate; “distant band” does so only at radius ≥2, preserving the original
home/immediate-neighbor search. Every selected candidate still requires exact
element packing. “Guide” refreshes weights using current Outline setup slack.

| Trial | 100×100 packed WNS | Packed TNS | 50×50 packed WNS | Packed TNS |
|---|---:|---:|---:|---:|
| Baseline | −3.759 | 2917.478 | −0.884 | 13.628 |
| Guide only | −3.714 | 3373.333 | −0.954 | 11.623 |
| Band 2 | −3.174 | 1279.480 | −0.744 | 9.659 |
| Guide + band 2 | −3.239 | 1711.360 | −0.924 | 11.013 |
| Band 8 | −3.184 | 1682.300 | −0.884 | 12.086 |
| Guide + band 8 | −3.469 | 2227.090 | −1.044 | 14.054 |
| Fractional ordering within target Tile | −3.777 | 3330.360 | −0.909 | 15.328 |
| Globally fractional y/x ordering | −3.449 | 2621.070 | −0.839 | 13.795 |
| Distant band 2 | −3.179 | 1310.470 | −0.634 | 9.844 |
| Distant band 4 | −3.524 | 1479.810 | −0.879 | 9.594 |
| Fractional within Tile + distant band 2 | −2.982 | 1318.050 | −1.032 | 10.241 |
| Globally fractional + distant band 2 | −3.654 | 1072.980 | −0.919 | 4.846 |
| Squared delay, original rings | −3.807 | 2316.130 | −0.884 | 12.917 |
| Cubic delay, original rings | −3.797 | 2600.810 | −0.884 | 11.674 |
| Squared delay + distant band 2 | −2.649 | 781.300 | −0.604 | 5.356 |
| **Cubic delay + distant band 2 (selected)** | **−2.624** | **787.730** | **−0.589** | **3.600** |

The combination matters: merely changing the score does not fix the search
stopping before a better candidate can be considered. Fractional ordering and
refreshed weights had mixed/worse results and were discarded. The squared
combination has marginally better 100×100 TNS, but cubic gives better WNS on
both sizes and better 50×50 TNS.

## Retained change

Only `PlaceDesign::preSmearBunches()` changes:

1. Keep home and immediate-neighbor reservations local. At radius ≥2, compare
   two extra rings, without changing the search limit or fallback rules.
2. Rank candidates by `sum(connection_weight * calibrated_wire_delay^3)`.
   This prevents several short connections from cheaply outvoting a much
   longer one. Equal scores prefer the nearer radius, then the existing
   right/down preference.

Existing linear bunch traversal, whole-bunch shape, live resource previews,
fixed-cell guards, input limits and exact reservation/commit validation remain
unchanged. This is a heuristic candidate score, not a claim that cubic wire
cost exactly models whole-path setup slack. No per-candidate timing traversal,
new scheduling layer or persistent cache was introduced.

On 100×100, exact packing calls remain 106,713, and precise per-Tile trials
remain 155,916. Scanned origins rise from 55,817,525 to 60,693,329; cheap
scored candidates rise from 632,549 to 2,115,302. Observed PlaceDesign time
including internal refinement was 8.77 s baseline versus 11.03 s selected
in the scoring sweep. There is extra scoring work, not extra exact-pack calls.

## Regression checks

Two new cases in `src/tests/placing/place_timing.cpp`:

- A two-cell bunch has only a wrong-way free site on radius 2, but a legal
  timing-safe site on radius 4. The baseline fails; bounded look-ahead passes,
  preserves the bunch offset and does not move existing occupants.
- Within radius 1, linear total wire cost chooses a seven-Tile connection
  over a five-Tile connection to shorten two already-short connections. The
  linear policy fails; the selected score passes all three setup endpoints.

The first fails on the original policy. Enabling only the band passes that
case but fails the second. The combined policy passes both and the existing
placement timing suite, including nearest-ring, fixed-I/O, input-capacity,
coupled-resource and preview-cleanup checks.

Full 50×50 passes: final WNS −0.099 ns, TNS 1.003 ns, no actionable endpoints
below −0.1 ns. The previous result was −0.082 ns / 1.295 ns, so final WNS is
slightly worse inside that repair stopping band, while TNS is lower. Swapping
attempts decrease from 278,072 to 56,769. The 12×12 single- and two-clock
puzzles also pass.

All 32 focused regressions passed (placement timing, Outline geometry and
attractors, multi-clock behavior and incremental PnR). After removing the
experiment switches, the 31 short/affected checks were rerun successfully;
the unchanged large Outline case had passed in 154.24 s. The final default
50×50 run reproduces the selected experiment's stage WNS/TNS values.

## Full 100×100 flow

| Stage | Reference WNS | Selected WNS | Reference TNS | Selected TNS |
|---|---:|---:|---:|---:|
| Outline | −0.342 | −0.342 | 2.862 | 2.862 |
| Exact packing | −3.759 | −2.624 | 2917.478 | 787.730 |
| Internal refinement | −3.759 | −2.624 | 2869.573 | 764.397 |
| PlaceTiming | −3.759 | −2.624 | 2660.227 | 700.233 |
| Sorting | −2.429 | −1.314 | 1589.416 | 328.825 |
| Swapping | −0.832 | −0.391 | 1321.604 | 153.637 |

The selected run uses the exact policy tested in the sweep. The reference log
is `build/outline_slack.SQr6AR/approved_strength2_full_100.log`; selected log
is `build/prepack_experiments.u4lCMw/full_cubed_100.log`.

Sorting finishes in 47.51 s without hitting its 60-second limit. Swapping
performs ten improving passes, accepts 935 swaps, then stops at the recovery
reserve (143.26 s used of 156.88 s available; `timed_out=0`). The test returns
failure because −0.391 ns is below its unchanged −0.300 ns acceptance limit.
The full flow was not declared passing and its tolerance was not changed.

After reviewing this result, the user approved a temporary −0.400 ns final
acceptance limit for 100×100 puzzles only. Smaller puzzles retain −0.300 ns.
The recorded −0.391 ns result meets this new limit; that is a tolerance change,
not an additional timing improvement or a newly rerun puzzle result.

Outline, compilation and regressions overlapped during these experiments.
Sorting/Swapping have time budgets, so downstream rows are observed full-flow
results, not identical-work benchmarks. The packing comparison itself is
fixed-work, starts from identical Outline geometry, and checks legal packing.

## Remaining loss

The worst packed 100×100 endpoint is still `mesh_cell_114357`, with
`mesh_cell_111156 → mesh_cell_112765 → mesh_cell_114357`:

- Outline: `(48,78) → (47,78) → (47,81)`; edge distances 1 and 3.
- Packed: `(56,98) → (22,85) → (73,85)`; edge distances 47 and 51.

Therefore the selected change reduces the loss substantially but does not
eliminate distant separation of connected bunches. Maximum bunch translation
is still 33 Tiles. This is not a complete solution to dense-region placement.
