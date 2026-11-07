# Two-clock problems found and fixed

## Missing explicit clock-to-clock maximum-delay constraints

The timing engine supported primary-clock periods and asynchronous exclusions,
but no directional data-path delay budget. Cutting FIFO crossings would leave
the requested constraints unchecked. Add a validated directional constraint map
and `set_max_delay -datapath_only -from clock -to clock ns` Tcl support. The
timing forest expresses each limit at the launch leaf; every placement evaluator
then consumes the same constraint. Conflicting asynchronous cuts are rejected.
The focused regression checks both directions, invalid inputs, unchanged state
after rejection and a deliberately impossible limit that produces a violation.

## Negative launch offsets were clamped in RTL timing

A delay limit larger than the capture period needs a negative launch offset.
`Timings::recurseTimings` initialized its maximum at zero, discarding negative
arrivals through logic. Initialize at negative infinity instead. A 3 ns limit
into a 1.5 ns capture domain through a 0.2 ns LUT must retain an RTL arrival of
−1.3 ns. Full, prepared and local placed-timing checks agree on the physical
delay and slack after moving that LUT.

## Sorting's trial predictor omitted launch offsets

Sorting's direct coordinate predictor returned zero at launch leaves even
though committed timing included the clock relationship. Preserve
`launch_offset_ns` in trial calculations, matching its tied-path predictor and
the full/local/prepared evaluators. Dedicated tests exercise Sorting and
Swapping repairs under explicit cross-clock maximum delays.

## Flattened hierarchy introduced metadata cells

The two-domain wrapper produces Yosys `$scopeinfo` hierarchy metadata. These
are not physical cells. Delete this specific metadata type in the synthesis
script before counting/importing cells. Unknown functional primitives still
cause a failure; no logic or register is filtered out of the count.

## Shared primary inputs introduced unintended arbiter crossings

The first 1k priority-arbiter variant drove both domains from permutations of
one input stream. Yosys shared logic between equivalent request predicates,
introducing physical register-to-register paths between domains. The independent
crossing audit rejected them even though the aggregate clock timing passed.
Give independent-domain workloads separate 16-bit input streams and export
both outputs directly. This avoids implicit CDCs without `keep` attributes or
false-path exceptions. The FIFO intentionally retains and checks both crossing
directions. Recalibrate all affected sources against actual mapped cells.

## Small-tier granularity

Two independent multiplier/reduction trees fell just below the 1k count band.
Register each tree's incoming sample before its multiplier map; this is a useful
pipeline boundary retained at every size. A FIFO bank is relatively coarse, so
the additional asynchronous family uses 12-bit payloads, representative of a
sample stream, with 8 entries per bank. No tolerance is relaxed or unused logic
added. The 1k asynchronous case contains three banks and 922 mapped cells.

## Final validation

All 77 two-clock designs pass, generated and validated one size tier at a time,
with 3,307 placement runs under one verified binary. All clock domains have
positive final slack; the smallest is 0.561 ns. The 100k FIFO's worst forward
delay is 4.825 ns against 6 ns, and its worst reverse delay is 4.628 ns against
7 ns. Both directions are exercised at every size. No clock or delay limit was
relaxed during calibration or placement.

The generated FIFO also passes the two-clock functional scoreboard, covering
data order, pointer wrap and full/empty operation. All 45 selected placement,
PnR, multi-clock and main Tcl regressions pass. The new Tcl command was also
checked through the rebuilt main executable's actual command registration.
See [RESULTS_2CLK.md](RESULTS_2CLK.md) for measured evidence and hashes.
