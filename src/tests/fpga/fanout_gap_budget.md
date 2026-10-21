# Detached-gap repair budget regression

Run both cases with:

```sh
ctest --test-dir build -R '^fpga.fanout_gap_(budget|failure_location)$' --output-on-failure
```

The fixture uses only abstract signal, cell, matrix and node names. It follows
the Example 1 failure's recorded prefix coordinates, disconnected tail and
two-JOINT Grounding. A real transit cut creates the 14-fragment partial route:

```
source (11,6)/LOCAL149 -> (15,6) -> (16,7) -> (16,8)/DST19
  GAP -> (5,6) -> retained tail -> (12,7)/DST67
  -> JOINT60 -> JOINT45 -> LOCAL107
```

The unrecorded fabric is synthetic, not an asserted full-device reproduction:
256 usable landings lead into a terminal-less region, and a different source
exit can reach the private sink input in one hop. The cut transit has a real
foreign Takeoff owner. All occupied resources are reconstructed from registered
routes and audited before and after each test. No bare busy bits are used.

Before the fix, the exact-gap FIFO explores the irrelevant region until a
deterministic 200-poll cancellation budget expires. It never tries the available
source bypass. The regression fails with 201 polls. An already-expired production
deadline also reports the retained sink `(12,7)/DST:-1`, not the unresolved gap.

The repair now probes the existing whole-source algorithm after 64 local states,
with at most 1,024 states and no preemption. If that probe fails, the local search
resumes and the original full source fallback remains available. Neither the
caller limits nor preemption eligibility are relaxed. The successful case takes
74 cancellation polls and completes the one-hop route without changing the
foreign owner. Controls check immediate cancellation, unreachable input, state
and hop limits, unchanged leases after failed speculation, and retained sink-pin
ownership. Failure reporting identifies the gap boundary and explicitly includes
stage deadline/watchdog status.

This establishes the local search-starvation and reporting defects. It does not
prove that the original full-device failed binding had this synthetic bypass.

## Full-size early source probe (2026-10-14)

A later Example 1 replay completed the previously failing remote Takeoff, then
reached pass 23 with 193 tasks remaining before the unchanged 600-second stage
deadline. Scoped profiling attributed 477.6 seconds to exact-gap searches
(including their early probes). In 1,007 slow cases, exact search cost 383.6
seconds before source fallback cost 33.5 seconds. One binding spent 561 ms on
exact search after a failed small probe, then 12.7 ms on successful free source
search. This is a search-budget ordering issue, not proof of unroutable fabric.

`fpga.fanout_gap_probe_budget` extends the abstract fixture with 1,398 extra
source choices and 2,000 gap-frontier landings. The useful source exit is
excluded by the 1,024-state probe; the gap flood then consumes the work budget
before the same source search can run at full size. These counts are synthetic
and are not claimed as the real chip's topology. Names remain abstract, and
route ownership, cancellation, hop/state bounds and failed-search immutability
are checked as in the original fixture.

Before the change, the new test fails after 2,501 cancellation polls. Using the
caller's existing state budget for the early free-only search completes it in
1,472 polls. The existing exact-gap search and preemption-assisted fallback
remain available. No global state/hop limit or timeout is increased, and no
cross-attempt search cache is added. All 22 focused repair regressions passed;
the complete 142-test suite (including eleven puzzles) then passed in
2,171.14 seconds without relaxed thresholds or deadlines. The real replay is
evaluated separately; passing the regression suite alone is not end-to-end
success.

## Verification, 2026-10-08

Both new cases failed before the production fix and passed afterward. The full
current-tree CTest run rebuilt every target and completed 89 tests: 87 passed,
with these two failures (no timeout or slack tolerance was changed):

- `puzzle.routing_50k_90`: routing deadline, 298.50 seconds overall. Generation
  took 22.745 seconds and Basic took 244.425 seconds with zero preemptions.
  A separate control disabled the early probe and also timed out, at 300.10
  seconds. Its Basic traversal had exactly the same 343,399 pops and 295,398
  accepted trials; an explicit diagnostic at the suppressed probe recorded
  zero invocations. The timeout is not explained by this new probe.
- `puzzle.placing_100x100_2clocks`: completed in 584.36 seconds, but final clock-1
  WNS was -0.536 ns against the existing -0.400 ns tolerance; clock 0 reached
  -0.129 ns. Placement code was not changed by this routing fix.

The control edit was removed and the fixed source's SHA-256 verified identical
to the source used for the full suite. The 50K routing forest, 50x50 placement,
100x100 single-clock placement, all smaller multi-clock cases, and incremental
PnR regressions passed.
