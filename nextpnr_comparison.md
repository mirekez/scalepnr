# scalepnr and nextpnr-xilinx PnR Comparison

## Scope

This experiment compares scalepnr with
[gatecat/nextpnr-xilinx](https://github.com/gatecat/nextpnr-xilinx) on the
same randomly generated Artix-7 designs. The purpose is to measure complete
place-and-route process time and to verify that both tools finish with zero
unrouted nets. It is not a quality-of-results, timing-closure, power, or
bitstream comparison.

The test target is `xc7a100tfgg676-1`. Each case is generated once by the
patched `pnr_tests` generator and synthesized once by Yosys. The resulting
Yosys JSON and pin constraints are then supplied to both PnR tools. Generator
and Yosys time are excluded from both measurements. No Vivado process, Vivado
export, FASM export, bitstream generation, or design-database serialization is
included.

The campaign ran 10 seeds at each requested maximum combinational-chain limit:
10, 20, and 30. This gives 30 paired executions, or 60 PnR processes. The tool
execution order alternates by case to avoid consistently favoring the second
tool through process-order effects.

## Test Configuration

| Item | Value |
|:---|:---|
| Date | 2026-08-05 |
| Host | Linux 7.0.0-28-generic, x86-64 |
| CPU | AMD Ryzen 5 3500, 8 visible cores, one thread per core |
| Memory | 10 GiB RAM, 4 GiB swap |
| FPGA | `xc7a100tfgg676-1` |
| Random seeds | 700 through 709 for each chain cap |
| Pipeline size | 1 random `pnr_tests` processing stage |
| Generator complexity | 1 |
| Maximum datapath width | 64 bits |
| Chain caps | 10, 20, 30 |
| PnR frequency target | 200 MHz / 5 ns |
| Per-process timeout | 600 seconds, then 15-second forced-kill grace |
| Yosys | 0.52, commit `fee39a3284c90249e1d9684cf6944ffbbcbb8f90` |
| scalepnr source | commit `bb30219`, tested binary SHA-256 `d8ceabbec4d3b7bf593d90894d0f439711bec520d150e929fc3bc45b66372262` |
| nextpnr-xilinx | commit/version `8f178fc`, tested binary SHA-256 `927a51960b7e4301689146d8d2dbe76a3616162878606b113a6beed2433637d3` |
| nextpnr chip DB | generated from the local Project X-Ray database, SHA-256 `bd6e7fc9529a98057fec29803ef9af1b151e932c42a96a1c253238496a9fdb0b` |
| `pnr_tests` | commit `cc883e3` plus `pnr_tests_max_chain.patch` |

The measured interval is wall-clock time around each complete PnR executable.
It therefore includes that executable's architecture-database loading,
netlist loading, packing, placement, routing, and final legality checks. It
does not include shared design generation or synthesis. scalepnr runs with
`SCALEPNR_SKIP_WRITE_DESIGN=1`; nextpnr-xilinx runs without `--write` or
`--fasm`.

## Completion Checks

A zero exit code alone is not considered success.

scalepnr must report all of the following:

- `routeDesign stage report: reason=complete`;
- Moving sources finishes with zero trunk tasks;
- the final Moving destinations stage finishes with zero physical-route tasks;
- the mandatory Moving sources and terminal Moving destinations stages report
  `timeout=false`; Basic and Fanouts may use their bounded handoffs;
- clock routing reports `failed=0` when clock routing is required.

Basic trunks may be handed to Moving sources, but that mandatory source stage
must reach zero before Fanouts starts. Fanout tasks may then be handed to Moving
destinations. These are intermediate stage transfers, so the checker requires
the two Moving barriers, rather than every intermediate stage, to reach zero.

nextpnr-xilinx must report all of the following:

- its final router iteration has `overused=0` and `overuse=0`;
- the router1 legality check reports `Routing complete.`;
- the process reports `Program finished normally.`;
- the log contains no `ERROR:` line.

All 60 retained logs were independently rechecked after the campaign. The
result was 30 valid scalepnr logs and 30 valid nextpnr-xilinx logs.

## Aggregate Results

| Cap | Runs | Observed chain | Cells (median, range) | Nets (median, range) | scalepnr median / mean / max (s) | nextpnr median / mean / max (s) | Median ratio | Both zero-unrouted |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10 | 10 | 0-6 | 145.5 (70-1155) | 43.5 (15-228) | 68.708 / 102.995 / 422.835 | 2.301 / 2.901 / 6.614 | 30.41x | 10/10 |
| 20 | 10 | 0-6 | 144 (70-1155) | 30.5 (15-228) | 69.865 / 103.423 / 418.398 | 2.343 / 2.892 / 6.644 | 30.95x | 10/10 |
| 30 | 10 | 0-6 | 144 (70-1155) | 41 (15-228) | 69.243 / 103.455 / 425.415 | 2.341 / 2.923 / 6.540 | 30.31x | 10/10 |

Across all 30 pairs, scalepnr's median was `68.937 s` and mean was
`103.291 s`. nextpnr-xilinx's median was `2.338 s` and mean was `2.905 s`.
The median paired ratio was `30.48x`; the minimum and maximum paired ratios
were `9.45x` and `103.92x`. No process reached the 600-second timeout.

The larger mean relative to median is caused by seed 706. That design contains
1,155 synthesized cells, including 810 flip-flops, and takes approximately
seven minutes in scalepnr while remaining a four-second nextpnr-xilinx case.

## Per-Case Results

`LUT`, `FF`, and `MUX` are sums of the corresponding synthesized primitive
types. `IO/clock` includes input buffers, output buffers, and clock buffers.
`PASS, 0/0` means scalepnr and nextpnr-xilinx both completed with zero unrouted
nets.

| Cap | Seed | Chain | Cells | LUT | FF | MUX | IO/clock | Nets | Order | scalepnr s | nextpnr s | Ratio | Result |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---|---:|---:|---:|:---|
| 10 | 700 | 6 | 188 | 89 | 0 | 29 | 70 | 93 | scalepnr-first | 68.271 | 2.257 | 30.25x | PASS, 0/0 |
| 10 | 701 | 1 | 89 | 19 | 0 | 0 | 70 | 15 | nextpnr-first | 63.682 | 1.987 | 32.05x | PASS, 0/0 |
| 10 | 702 | 0 | 70 | 0 | 0 | 0 | 70 | 15 | scalepnr-first | 62.487 | 6.614 | 9.45x | PASS, 0/0 |
| 10 | 703 | 0 | 70 | 0 | 0 | 0 | 70 | 15 | nextpnr-first | 62.720 | 2.600 | 24.12x | PASS, 0/0 |
| 10 | 704 | 3 | 144 | 49 | 14 | 6 | 71 | 83 | scalepnr-first | 70.627 | 2.346 | 30.11x | PASS, 0/0 |
| 10 | 705 | 4 | 153 | 53 | 0 | 30 | 70 | 69 | nextpnr-first | 66.646 | 2.180 | 30.57x | PASS, 0/0 |
| 10 | 706 | 3 | 1155 | 272 | 810 | 1 | 71 | 228 | scalepnr-first | 422.835 | 4.187 | 100.99x | PASS, 0/0 |
| 10 | 707 | 3 | 144 | 49 | 14 | 6 | 71 | 83 | nextpnr-first | 70.799 | 2.396 | 29.55x | PASS, 0/0 |
| 10 | 708 | 1 | 147 | 17 | 59 | 0 | 71 | 18 | scalepnr-first | 69.145 | 2.235 | 30.94x | PASS, 0/0 |
| 10 | 709 | 1 | 154 | 20 | 63 | 0 | 71 | 18 | nextpnr-first | 72.735 | 2.210 | 32.91x | PASS, 0/0 |
| 20 | 700 | 6 | 188 | 89 | 0 | 29 | 70 | 93 | scalepnr-first | 68.729 | 2.272 | 30.25x | PASS, 0/0 |
| 20 | 701 | 1 | 89 | 19 | 0 | 0 | 70 | 15 | nextpnr-first | 64.490 | 1.993 | 32.36x | PASS, 0/0 |
| 20 | 702 | 0 | 70 | 0 | 0 | 0 | 70 | 15 | scalepnr-first | 63.091 | 6.644 | 9.50x | PASS, 0/0 |
| 20 | 703 | 0 | 70 | 0 | 0 | 0 | 70 | 15 | nextpnr-first | 66.498 | 2.594 | 25.64x | PASS, 0/0 |
| 20 | 704 | 3 | 144 | 49 | 14 | 6 | 71 | 83 | scalepnr-first | 71.965 | 2.326 | 30.94x | PASS, 0/0 |
| 20 | 705 | 3 | 97 | 26 | 0 | 1 | 70 | 17 | nextpnr-first | 64.553 | 2.051 | 31.47x | PASS, 0/0 |
| 20 | 706 | 3 | 1155 | 272 | 810 | 1 | 71 | 228 | scalepnr-first | 418.398 | 4.026 | 103.92x | PASS, 0/0 |
| 20 | 707 | 3 | 144 | 49 | 14 | 6 | 71 | 83 | nextpnr-first | 71.002 | 2.360 | 30.09x | PASS, 0/0 |
| 20 | 708 | 1 | 148 | 19 | 58 | 0 | 71 | 18 | scalepnr-first | 71.619 | 2.272 | 31.52x | PASS, 0/0 |
| 20 | 709 | 3 | 167 | 32 | 54 | 10 | 71 | 43 | nextpnr-first | 73.883 | 2.386 | 30.97x | PASS, 0/0 |
| 30 | 700 | 6 | 188 | 89 | 0 | 29 | 70 | 93 | scalepnr-first | 68.609 | 2.330 | 29.45x | PASS, 0/0 |
| 30 | 701 | 1 | 89 | 19 | 0 | 0 | 70 | 15 | nextpnr-first | 63.544 | 2.061 | 30.83x | PASS, 0/0 |
| 30 | 702 | 0 | 70 | 0 | 0 | 0 | 70 | 15 | scalepnr-first | 63.497 | 6.540 | 9.71x | PASS, 0/0 |
| 30 | 703 | 0 | 70 | 0 | 0 | 0 | 70 | 15 | nextpnr-first | 63.046 | 2.583 | 24.41x | PASS, 0/0 |
| 30 | 704 | 3 | 144 | 49 | 14 | 6 | 71 | 83 | scalepnr-first | 69.876 | 2.353 | 29.70x | PASS, 0/0 |
| 30 | 705 | 3 | 121 | 40 | 0 | 11 | 70 | 39 | nextpnr-first | 64.857 | 2.145 | 30.24x | PASS, 0/0 |
| 30 | 706 | 3 | 1155 | 272 | 810 | 1 | 71 | 228 | scalepnr-first | 425.415 | 4.310 | 98.70x | PASS, 0/0 |
| 30 | 707 | 3 | 144 | 49 | 14 | 6 | 71 | 83 | nextpnr-first | 70.099 | 2.307 | 30.39x | PASS, 0/0 |
| 30 | 708 | 1 | 148 | 19 | 58 | 0 | 71 | 18 | scalepnr-first | 71.162 | 2.244 | 31.71x | PASS, 0/0 |
| 30 | 709 | 3 | 167 | 32 | 54 | 10 | 71 | 43 | nextpnr-first | 74.444 | 2.354 | 31.62x | PASS, 0/0 |

## Interpretation

The comparison shows a clear runtime distinction without increasing design
size: nextpnr-xilinx is about 30 times faster at the median on this workload.
The result does not imply that the tools optimize the same objective or
produce routes of equal timing or wire length. It demonstrates only complete
PnR wall time and successful route completion on identical input designs.

The requested chain limits are maxima, not requested exact depths. For these
seeds, post-synthesis measured depths range from 0 to 6. Several seeds produce
the same RTL under multiple caps; there are 13 unique synthesized SV hashes in
the 30 rows. This reuse is intentional for paired cap comparisons and means
the 30 rows are repeated executions across three configurations, not 30
statistically independent netlists. There are still 10 independent random
seeds and a broad size range from 70 to 1,155 synthesized cells.

The baseline timing difference is visible in every case, including the
smallest 70-cell designs, and increases substantially on the 1,155-cell
design. A separate 10k+ cell stress campaign was subsequently run to measure
behavior beyond this original range; its results follow.

## 10k+ Cell Stress Campaign

The updated campaign replays ten distinct synthesized designs from the random
test corpus. Generation and Yosys are outside both timers. The table reports
the measured post-synthesis chain depth rather than treating the generator's
requested cap as the measured value. Both PnR tools received identical JSON
and constraints, alternated first-run order, and used the same 600-second
process-group timeout.

`Basic limit` means scalepnr reached its internal 300-second Basic-routing
budget and invoked its assertion exit (`FAIL_139` in `summary.tsv`). `Timeout`
means the outer comparison process killed the complete process group after
600 seconds. Neither status is a successful or zero-unrouted result.

| Case | Seed | Chain | Cells | Nets | FF | LUT | MUX | scalepnr | nextpnr-xilinx |
|---:|---:|---:|---:|---:|---:|---:|---:|:---|:---|
| 1 | 801 | 8 | 11,667 | 3,195 | 8,101 | 2,877 | 124 | 470.220 s, Basic limit (19,833 -> 6,884) | 82.434 s, pass, 0 unrouted |
| 2 | 801 | 10 | 16,183 | 4,625 | 11,067 | 4,050 | 232 | 600.204 s, timeout (27,480 -> 10,626) | 190.551 s, pass, 0 unrouted |
| 3 | 804 | 13 | 15,649 | 7,241 | 7,085 | 6,330 | 1,052 | 513.050 s, Basic limit (21,730 -> 7,564) | 99.994 s, pass, 0 unrouted |
| 4 | 801 | 17 | 10,000 | 4,330 | 4,773 | 3,917 | 63 | 465.907 s, Basic limit (15,972 -> 5,288) | 600.213 s, timeout |
| 5 | 823 | 19 | 10,352 | 4,619 | 5,982 | 3,234 | 35 | 515.870 s, Basic limit (16,687 -> 6,372) | 52.180 s, pass, 0 unrouted |
| 6 | 810 | 24 | 10,298 | 4,349 | 5,850 | 3,137 | 32 | 492.619 s, Basic limit (16,667 -> 6,534) | 51.464 s, pass, 0 unrouted |
| 7 | 811 | 23 | 12,203 | 4,917 | 7,071 | 3,729 | 69 | 548.713 s, Basic limit (20,414 -> 8,331) | 88.211 s, pass, 0 unrouted |
| 8 | 802 | 24 | 12,013 | 4,719 | 6,125 | 4,835 | 96 | 457.216 s, Basic limit (18,977 -> 7,495) | 600.123 s, timeout |
| 9 | 800 | 22 | 14,221 | 4,351 | 8,953 | 3,857 | 49 | 600.393 s, timeout (24,643 -> 10,207) | 94.621 s, pass, 0 unrouted |
| 10 | 802 | 37 | 15,664 | 4,548 | 8,838 | 5,880 | 234 | 586.316 s, Basic limit (24,308 -> 8,913) | 147.806 s, pass, 0 unrouted |

The placement performance fix is confirmed across the full set: every
scalepnr run completed placement and entered Basic routing. The previous
five-case campaign never reached a routing-stage report. The remaining
scalepnr scaling boundary is now Basic-routing convergence, not placement.
Scalepnr passed 0/10 complete PnR runs; nextpnr-xilinx passed 8/10 with zero
unrouted nets and timed out on two. Among its eight successful runs,
nextpnr-xilinx had a mean of 100.908 seconds and median of 91.416 seconds.
Scalepnr's median stopping time was 514.460 seconds, but no speed ratio is
reported because none of its runs completed.

Raw logs, per-tool memory files, the exact netlist manifest, and the canonical
machine-readable results are stored under:

```text
tests/prjxray/nextpnr_runs/optimized_10k_20260806/
```

## 50k+ Cell Stress Campaign

The 50k+ campaign contains ten distinct post-Yosys designs between 50,000 and
60,000 cells. Four cases use maximum generated chain length 10, three use 20,
and three use 30. The measured chain depth is reported separately because the
limit constrains generation; it does not force every random design to reach
that depth. All ten synthesized-system-Verilog hashes are unique.

The runner adaptively changes the requested generator stage count until the
post-synthesis cell count is in range. A candidate is retained only when Yosys
succeeds, the size check passes, and `check_chain_length.py` proves that the
measured register-to-register combinational depth does not exceed its cap.
Generation, rejected candidates, and Yosys remain outside both PnR timers.

For fairness, nextpnr-xilinx is invoked with
`--placer-heap-cell-placement-timeout 0`. The option is an internal placement
attempt divisor, not a wall-clock timeout; leaving its default enabled caused
premature placement failures. Both tools instead use the same external
600-second process-group watchdog and 15-second forced-kill grace.

`Basic limit` denotes scalepnr's intentional assertion after its 300-second
Basic-routing stage budget. `Process timeout` denotes the common external
watchdog. The parenthesized scalepnr figures are Basic tasks at stage entry and
at the last complete stage report. For externally killed runs, `last todo` is
the final heartbeat and is not presented as a completed-pass count.

| Case | Cap | Seed | Chain | Cells | Nets | FF | LUT | MUX | scalepnr | nextpnr-xilinx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---|:---|
| 1 | 10 | 5000 | 3 | 51,752 | 16,651 | 35,620 | 13,885 | 354 | 600.241 s, process timeout (51,048 -> 39,794 before state dump) | 600.262 s, process timeout |
| 2 | 10 | 5001 | 3 | 50,680 | 17,142 | 34,340 | 13,936 | 445 | 585.877 s, Basic limit (49,804 -> 38,506) | 600.162 s, process timeout |
| 3 | 10 | 5006 | 3 | 50,401 | 17,537 | 33,820 | 13,756 | 361 | 592.111 s, Basic limit (49,684 -> 36,552) | 600.184 s, process timeout |
| 4 | 10 | 5007 | 3 | 56,033 | 18,789 | 38,009 | 15,333 | 454 | 594.139 s, Basic limit (55,135 -> 40,913) | 600.192 s, process timeout |
| 5 | 20 | 6001 | 3 | 52,717 | 17,548 | 35,938 | 14,384 | 438 | 600.182 s, process timeout (last todo 39,999) | 600.182 s, process timeout |
| 6 | 20 | 6002 | 3 | 51,203 | 17,287 | 34,681 | 13,993 | 383 | 570.133 s, Basic limit (50,453 -> 37,881) | 600.217 s, process timeout |
| 7 | 20 | 6003 | 3 | 51,227 | 17,351 | 34,659 | 14,048 | 403 | 550.140 s, Basic limit (50,444 -> 37,211) | 600.123 s, process timeout |
| 8 | 30 | 7001 | 26 | 53,881 | 21,645 | 29,832 | 19,693 | 197 | 600.199 s, process timeout (last todo 41,017) | 600.108 s, process timeout |
| 9 | 30 | 7003 | 26 | 55,583 | 23,251 | 29,279 | 21,614 | 125 | 578.331 s, Basic limit (55,339 -> 43,779) | 600.187 s, process timeout |
| 10 | 30 | 8000 | 25 | 57,439 | 23,086 | 30,721 | 21,782 | 328 | 569.288 s, Basic limit (56,853 -> 39,545) | 600.195 s, process timeout |

The median design has 52,234.5 cells and 17,542.5 nets. The full ranges are
50,401-57,439 cells and 16,651-23,251 nets. scalepnr completed placement and
entered Basic routing in every case, but completed zero full PnR runs within
the limit. Seven runs stopped at the internal Basic budget and three at the
outer watchdog. Its median stopping time was 588.994 seconds. nextpnr-xilinx
also completed zero full PnR runs: all ten were still in analytical placement
at the outer timeout, for a median stopping time of 600.186 seconds. A speed
ratio would be misleading because neither tool produced a completed sample.

The scalepnr profiling performed while preparing this campaign identified and
fixed several avoidable costs without changing routing policy:

- placement candidate lookup now uses spatial/type buckets and cached outline
  topology rather than repeatedly scanning unrelated cells;
- tile input-to-joint reservation masks are cached instead of reconstructed
  for every route attempt;
- route endpoint preparation is cached per task and invalidated when a task is
  retargeted;
- expensive route-name and preemption diagnostics are disabled at normal log
  verbosity;
- nextpnr's unrelated internal placement-attempt limit is disabled.

On a fixed 53,976-cell profiling design, caching endpoint preparation improved
first-minute Basic-task throughput from 10,647 to 12,517 tasks (17.6%), while
the input-to-joint cache reduced the measured setup portion from roughly 72
seconds to roughly 7 seconds. These changes are material, but the campaign
shows that Basic-routing convergence remains scalepnr's limiting stage at
50k+ cells. For nextpnr-xilinx, analytical placement is the limiting stage.

### Post-campaign 50k profile

A fresh paired run of seed 5000 was made after removing Moving-only
source-tree reset scans from Generic and Fanout task batches. The regenerated
design contains 51,749 cells and uses chain cap 10. scalepnr stopped at its
300-second Basic-stage budget after 588.446 seconds of complete-process wall
time; nextpnr-xilinx reached the common 600-second watchdog while still in
analytical placement. Peak resident memory was approximately 3.87 GiB for
scalepnr and 0.64 GiB for nextpnr-xilinx, both below the 6 GiB process limit.

The same saved scalepnr placement was then replayed to isolate routing. The
first Basic pass improved from 59.014 seconds to 41.740 seconds. In a
60-second profile, unfinished Basic tasks improved from 40,155 before the
change to 35,514 after it. A complete 300-second Basic profile produced the
following pass history:

| Basic pass | Elapsed routing s | Tasks before | Tasks after | Completed | Successful preemptions |
|---:|---:|---:|---:|---:|---:|
| 1 | 41.740 | 51,048 | 40,155 | 10,893 | 0 |
| 2 | 60.431 | 40,155 | 35,212 | 4,943 | 0 |
| 3 | 88.120 | 35,212 | 34,529 | 686 | 2,241 |
| 4 | 123.027 | 34,529 | 34,201 | 358 | 2,042 |
| 5 | 161.562 | 34,201 | 33,850 | 517 | 3,008 |
| 6 | 205.616 | 33,850 | 33,556 | 559 | 3,210 |
| 7 | 257.214 | 33,556 | 33,459 | 672 | 4,123 |
| 8 | 300.497 | 33,459 | 33,671 | 664 | 5,381 |

Across these passes scalepnr completed 19,292 tasks, performed 20,005
successful preemptions, and removed 91,627 fragments from victim routes. The
task count increasing in pass 8 demonstrates that displaced completed trunks
are being requeued faster than congestion is resolved. The remaining 50k
bottleneck is therefore negotiated-congestion convergence, not memory use,
device-database loading, or per-task source-tree scanning.

A second profile kept the first-pass takeoff sweep but changed later Generic
passes to visit committed prefixes before takeoffs and empty routes. At the
300-second limit it left 33,179 tasks, 492 fewer than source-first ordering,
and had completed 20,191 tasks instead of 19,292. This is a modest 1.5% residual
improvement, not route completion. It also confirms the congestion diagnosis:
the run still performed 20,954 successful preemptions and removed 104,165
victim fragments. This progress-first ordering is retained because it improves
the measured result without changing numeric route search or first-pass source
reservation.

### Ten-minute timeout rerun

The 50k campaign was repeated with a 600-second budget for each scalepnr
routing stage and a 1,200-second complete-process guard. It was configured to
stop immediately at the first scalepnr timeout. The first case, chain cap 10
and seed 5000, contains 51,752 synthesized cells, 16,651 nets, and measured
chain depth 3. Basic routing timed out after 600.045 routing seconds, so later
cases and nextpnr were intentionally not run.

| Metric | Result |
|:---|---:|
| Complete scalepnr process time | 882.345 s |
| Peak resident memory | 4,108,904 KiB |
| Basic tasks | 51,048 -> 35,633 |
| Basic passes | 15 |
| Task attempts | 522,676 |
| Completed task events | 31,809 |
| Successful preemptions | 67,131 |
| Completed victims preempted | 16,394 |
| Partial victims preempted | 50,737 |
| Victim fragments removed | 429,370 |

The best residual was 32,920 after pass 5. Continued routing then increased
the unfinished count through 32,993, 33,282, 33,563, 34,044, 34,400, 34,993,
35,301, 35,667, and 36,054 before the deadline-truncated final pass ended at
35,633. More time therefore does not solve this case: completed and partial
routes are displaced faster than stable new trunks are established.

The ten-minute artifacts and machine-readable result are retained at:

```text
/tmp/scalepnr_50k_10min/size_132_seed_5000_20260811_190242_321626/
```

The fresh paired artifacts are retained at:

```text
/tmp/scalepnr_50k_comparison_current/size_132_seed_5000_20260810_224916_244634/
```

The isolated post-fix replay is retained in
`/tmp/scalepnr_50k_batch300_guarded.log` with resource measurements in
`/tmp/scalepnr_50k_batch300_guarded.time`. The progress-first replay is in
`/tmp/scalepnr_50k_prefix300.log`, with measurements in
`/tmp/scalepnr_50k_prefix300.time`.

The combined machine-readable summary is stored at
`tests/prjxray/nextpnr_runs/optimized_50k_20260807/summary.tsv`. The replayable
designs and complete raw logs are retained under these roots:

```text
/tmp/scalepnr_50k_campaign_final/size_132_seed_5000_20260807_165236_312928/
/tmp/scalepnr_50k_campaign_final/size_132_seed_6000_20260807_183226_338993/
/tmp/scalepnr_50k_campaign_final/size_188_seed_8000_20260807_205339_364006/
```

## Reproduction

Run setup once:

```bash
tests/prjxray/prepare.sh
```

Run the same 30-case campaign:

```bash
SCALEPNR_COMPARE_ITERATIONS=10 \
SCALEPNR_COMPARE_CHAIN_LENGTHS="10 20 30" \
SCALEPNR_COMPARE_TIMEOUT=600 \
tests/prjxray/.run_nextpnr_test.sh 1 700
```

Generate a fresh ten-design-per-cap campaign requiring at least 10,000 cells:

```bash
SCALEPNR_COMPARE_ITERATIONS=10 \
SCALEPNR_COMPARE_CHAIN_LENGTHS="10 20 30" \
SCALEPNR_COMPARE_MIN_CELLS=10000 \
SCALEPNR_COMPARE_MAX_CANDIDATES=1000 \
SCALEPNR_COMPARE_TIMEOUT=600 \
tests/prjxray/.run_nextpnr_test.sh 50 900
```

Generate ten 50k-60k designs split 4/3/3 across the three chain caps:

```bash
SCALEPNR_COMPARE_ITERATIONS=10 \
SCALEPNR_COMPARE_CHAIN_LENGTHS="10 20 30" \
SCALEPNR_COMPARE_CASES_PER_CHAIN="4 3 3" \
SCALEPNR_COMPARE_MIN_CELLS=50000 \
SCALEPNR_COMPARE_MAX_CELLS=60000 \
SCALEPNR_COMPARE_MAX_CANDIDATES=1000 \
SCALEPNR_COMPARE_TIMEOUT=600 \
tests/prjxray/.run_nextpnr_test.sh 132 5000
```

The original small-design campaign data are under:

```text
tests/prjxray/nextpnr_runs/size_1_seed_700_20260805_131450_110055/
```

The updated ten-case 10k+ results are under:

```text
tests/prjxray/nextpnr_runs/optimized_10k_20260806/
```

The canonical machine-readable result is `summary.tsv`. Every failed or timed
out case retains its seed, generated design, constraints, tool logs, timing
file, and replayable directory. The script returns nonzero if either tool
times out, exits unsuccessfully, or fails its zero-unrouted validation.
