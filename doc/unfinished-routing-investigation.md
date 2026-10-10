# Unfinished routing tasks: 2,000-cell example 1 investigation

Diagnostic replay: `tests/prjxray/1/build/run.9j9z4eji` (2026-10-01).
Reference execution: `tests/prjxray/1/build/run.fS7O6W5d`.
No routing or packing behavior was changed for this investigation.

The replay enabled the existing task/continuation diagnostics and handoff state
dump. It reproduced the same first batch (1,989 cells), 49 Fanouts passes,
344 active unfinished tasks, 678 bridge preemptions, and final Moving failure.
Fanouts took 236.844 seconds with the extra diagnostics versus 220.731 seconds
in the reference. Neither execution hit the stage timeout.

## 1. A demonstrated extend/backstep retry loop

Route:
`$abc$157062$lut$aiger157061$15061.rv32m_inst.bext[33]`.

The trace repeatedly shows:

1. Continue from `(21,4)/DST3074`, route size 25.
2. Accept one incomplete hop to `(16,4)/DST3841`, route size 26.
3. The next continuation returns `root_blocked=true`.
4. Remove that hop, returning to size 25 with the same branch offset.

Four consecutive occurrences end at replay log lines 25412, 25535, 25652,
and 25746. Lines 25531–25536 contain one complete sequence. These are actual
repeated decisions, not an inference from unchanged task counts.

The relevant code is in `src/pnr/route/RouteDesign.cpp`:

- Around line 6302, failed docking can fall back to the first docking candidate
  even when there is no `partial_fallback_step` proven to have a continuation.
- Around lines 10393–10396, extending the vector resets the branch retry count.
- Around line 10432, removing a blocked hop also resets that count. The rejected
  committed edge is not retained as a task-local alternative already tried.

Consequently the four-attempt branch-rotation limit does not bound this
extend/backstep cycle. Persistent Basic deadends are intentionally disabled in
Fanouts; enabling them globally is not the appropriate correction.

Recommended correction: do not commit a dead docking fallback after docking
has failed, and preserve task-local failed-edge/alternative progress across a
committed backstep. A regression should hold occupancy constant and prove that
the next attempt tries another edge or exhausts the branch, rather than
recommitting the same failed hop.

## 2. Long private partial routes occupy resources without completing

The handoff snapshot is `/tmp/scalepnr_fanout_blocked_state.txt`.
Counts below exclude intra-Tile fragments and distinguish private suffixes
from copied shared prefixes.

| Route suffix identifying the task | Private inter-Tile hops | Repeated private Tile arrivals |
| --- | ---: | ---: |
| `$15061.rv32m_inst.bext[33]` | 79 | 18 |
| `reintegrate$157384…reintegrate$157380.A[2]` | 158 | 58 |
| `reintegrate$157388.rv32m_inst.aext[54]` | 189 | 39 |

Full names for the latter two:

- `$auto$abc_ops_reintegrate.cc:655:reintegrate$157384.$techmap158199$auto$abc_ops_reintegrate.cc:655:reintegrate$157380.A[2]`
- `$auto$abc_ops_reintegrate.cc:655:reintegrate$157388.rv32m_inst.aext[54]`

The 158-hop private suffix starts at `(16,39)`, four coordinate units from its
target CB `(12,39)`, but ends at `(12,5)`, 34 units away. It visits `(5,45)`
five times. This same unfinished route owns SRC261 at `(16,15)`, one of the
exits blocking the final Moving search.

`continuePartialRoute()` starts a fresh bounded search without passing the
retained prefix's Tile history. Each search is short, but their concatenation
is not bounded by that short-search limit. Vector growth counts as progress
and resets retries even when it merely extends a detour.

This establishes resource-consuming repeated traversal. It does not prove that
every repeated Tile is avoidable, or that a shorter complete route currently
exists. A correction needs route-aware repetition control/alternative search,
not an assertion that Manhattan distance must decrease at every hop.

In the reference run's final ten Fanouts passes there were 4,280 partial
advances, 1,199 successful backsteps, and 46 completed task attempts. Summed
Docking time was 159.864 seconds over the entire 220.731-second Fanouts stage.

## 3. Moving omits a retained partial sibling anchor

The ultimately failing task is:
`$abc$157062$lut$aiger157061$6595.$techmap160332$abc$157062$lut$aiger157061$6595.A[4]`.

Its physical driver is `$scalepnr_passthrough$604/O`. Basic completed that
driver's seed to `$abc$157062$lut$aiger157061$6598/I1`. Fanouts preemption event
10 correctly cut the seed from seven fragments to two, retaining:

`(15,14)/LOCAL159 -> SRC16 -> (15,15)/DST19`.

The handoff snapshot records that two-fragment route under net
`$auto$bufnorm.cc:387:execute$162207`, binding 0 (line 8968; fragments at
lines 87809–87810). Its route is incomplete, but the retained prefix is still
source-connected.

Moving's anchor collection around `RouteDesign.cpp:13265` includes the current
task's partial route and **completed** sibling routes. It excludes this other
incomplete binding. The failed search's `forward_search.txt` consequently has
only the LOCAL159 anchor, regards SRC16 as busy, and never seeds `(15,15)/DST19`.

This is a search-coverage gap. Moving should inspect source-connected retained
prefixes of incomplete siblings too, verifying ownership and preserving the
sibling's leases when branching. The existing snapshot proves the omitted
prefix; it does not prove that searching from it will find a complete solution.

## 4. The candidate rejection itself is supported by ownership evidence

Moving reaches `(16,15)/DST259` from the source. All eight modeled outgoing
SRC alternatives there are occupied: 19, 67, 244, 261, 262, 547, 739, 1030.
Four (19, 67, 261, 547) have only incomplete route owners. Across that CB,
21 leased SRCs and 22 leased DSTs have no completed route owner.

The candidate report audits 193 leased resources with zero unowned leases,
missing leases, missing registrations, or stale registrations. Multiple claims
examined there belong to the same physical driver, not conflicting drivers.

At resource Tile `(17,15)` all eight LUT positions were tried:

- Position 3 is occupied.
- Positions 7, 15, 135, 143 fail the packing chain check.
- Positions 11 and 139 fit and ground the trigger, but cannot reserve input I2.
  Their terminal reports contain respectively 48 occupied alternatives and
  46 occupied alternatives plus two unsupported incoming mappings.
- Position 131 fits but has no terminal entry from the reached DST to pin 89.
  Its summary incorrectly repeats the previous I2 failure string; the precise
  `GROUNDING_REJECT` line gives the actual cause.

Files: `routing_failure_candidates_001/forward_search.txt` and
`002_cb_16_15_dst_259_resource_17_15.txt` in either run directory.

## Handoff and remaining uncertainty

The 344 active tasks are not all remaining work: 247 tasks were already deferred
to Moving, giving 591 at the handoff. `fanoutShouldHandOff()` switches after at
least `max(32, recursion_limit*8)` passes when a pass completes at most two
tasks. The log's "blocked ... after 0 stagnant attempts" hides this specific
low-completion trigger; it is not evidence that all alternatives were exhausted.

Later, shared-prefix repair finds missing ownership at `(16,14)/SRC3616`,
invalidates 28 routes of another source, and starts 79 repair tasks including
previously deferred work. That is another recorded correctness concern; this
investigation has not isolated the mutation that caused that ownership loss.

Priorities are the demonstrated retry loop and omitted partial-sibling anchor,
followed by controlling repeated partial-route growth and tracing the separate
shared-prefix ownership loss. Increasing timeouts alone addresses none of them.

## Follow-up: persistent multilevel Fanouts search

The retry-loop correction adds `RouteSearchStack` to each task. Each committed
hop pushes a level; popping a blocked child retains its parent's attempted SRC
choices. The last private hop can be popped back to the shared branch root,
which explores its remaining children before the branch offset advances.
Changed prefixes invalidate descendant levels, and a changed destination resets
the search. This history is independent of Basic's global deadend masks.

The real-router regression `fpga.fanout_search_stack` suspends at every scheduler
boundary, pops two private levels, and checks both alternate-child completion
and exhaustive root rotation. Both cases run near and far from the target to
cover docking fallback. It also audits resource ownership after backtracking.
Clearing the stack at every attempt in a temporary fault-injection run makes the
alternate-child assertion fail; the fault injection was removed. The routing,
moving, bridge-preemption, docking, and search-stack CTest suites all pass.

Example replay: `tests/prjxray/1/build/run.xgfqRDmr`, same requested 2,000-cell
batch size (actual first batch 1,989), with task tracing and preemption audit.

| Fanouts metric | Prior reference | Persistent levels |
| --- | ---: | ---: |
| Time | 220.731 s | 159.602 s |
| Passes | 49 | 42 |
| Numeric searches | 44,772 | 32,432 |
| Active tasks at handoff | 344 | 389 |
| Total tasks entering Moving | 591 | 627 |
| Bridge preemptions | 678 | 632 |

All 632 replacement routes committed and completed; the audit reports no
historical cycles, immediate reversals, bad releases, or post-cut commit
failures. Basic still leaves one trunk and source Moving completes it. No stage
times out. The run is **not successful**: destination Moving fails for
`$abc$157062$lut$aiger157061$16096` (LUT6, old position `(23,10)/11`). Its forward
search has nine anchors and reaches 27 states; 46 candidate probes comprise 27
without a usable resource Tile and 19 packing rejections. Candidate reports are
under that run's `routing_failure_candidates_001/` directory.

Thus the repeated-child regression is fixed and fewer searches execute, but
overall completion has not improved. The earlier partial-sibling anchor and
long-detour findings were not changed in this correction.

## Follow-up: incomplete sibling anchors in Moving

Destination Moving now supplements its existing seeds with incomplete bindings
from the same physical driver/output. This lookup includes the task Net and
indexed sibling Nets, with a module lookup for callers without a source index.
It does not change the complete-seed predicates used by Basic/Fanouts.

New sibling prefixes are inspected once before candidate search. They must
start at the driver's mapped output, remain contiguous, resolve each hop to the
recorded landing, and have live leases with owners from the same physical
source tree. Only the connected leading portion is exposed as branch points.
Bindings whose source or sink belongs to the moving cluster are excluded. The
existing commit path copies the selected prefix as shared fabric; it neither
unroutes nor re-leases the sibling's takeoff.

`fpga.moving_incomplete_sibling` calls the production relocation function with
an empty suffix and an incomplete sibling owning the only usable takeoff. Before
the correction, the positive fixture failed with `scanned=0`. It now grounds
the suffix. The 48 cases cover same-Net/cross-Net bindings, indexed/unindexed
lookup, both branch-removal orders, wrong driver/port, missing source/landing
leases, detached or ownerless prefixes, co-moved sinks, conflicting ownership,
invalid hop topology, and packing rejection. Success and cleanup are checked
with the congestion ownership audit; rejection must preserve placement and
leases. This fixes the demonstrated search-coverage gap, not a claim that the
historical example's omitted anchor guarantees a complete route. The full
example has not been rerun for this correction.

## Follow-up: whole-path Tile repetition control

Fanouts now supplies the retained route to each numeric continuation, including
the shared trunk prefix on the initial branch attempt. A search seeds its
path-local visit tree once from that route; speculative children receive their
own immutable history roots. Resuming after a pop/truncation rebuilds from the
shortened route, so abandoned visits do not survive. A third Tile arrival is
rejected even when it uses a different, free DST. Adjacent same-CB transitions
are not additional visits. Path-limit failures do not create persistent or
search-wide Tile deadends.

Grounding and I/O Docking receive the retained history plus the speculative
path to their selected anchor. Both frontiers account for that prefix, and a
joined candidate is checked as a whole. The same check gates joined bridge
proposals and blocked-terminal reachability before either can authorize
preemption; rejecting only after a victim was cut would be too late.

Four production Fanouts cases cover near/far targets, multi-level backtracking,
legal alternatives (including an initially outward detour), exhaustion, and
lease audits. Fifteen Docking cases cover prefix-boundary revisits, same-CB
steps, bypasses, bridge/terminal blockers, and joins whose halves individually
pass but whose combined history fails. Their standalone baseline exposes the
route or preemption opportunity that the retained history must reject.

The routing, moving, bridge-preemption, search-stack, Fanouts Tile-visit,
incomplete-sibling, and docking CTest suites all pass (23.64 seconds total).
The main executable builds. This limits repetition, not total hops through
distinct Tiles. Example 1 has not yet been rerun with this correction.

## Combined replay: faster search, no end-to-end improvement

Replay `tests/prjxray/1/build/run.QZOhqEam` uses all three corrections above,
requested batch size 2,000 (actual first batch 1,989), unchanged 600-second
stage budgets, preemption audit, task tracing, and handoff snapshots. Its
`design.json` and `constraints.tcl` hashes match `run.9j9z4eji`. That earlier
diagnostic replay has the same routing decisions as `run.fS7O6W5d`, but includes
the snapshot overhead, making it the closer runtime comparison.

| Metric | Before the three corrections | Combined replay |
| --- | ---: | ---: |
| Placement time | 52.660 s | 60.973 s |
| Clock routing | 15.322 s | 15.821 s |
| Constant routing | 0.092 s | 0.092 s |
| Basic routing | 7.933 s | 8.613 s |
| Basic unfinished trunks | 1 | 1 |
| Source Moving | 0.007 s | 0.007 s |
| Fanouts time | 236.844 s | 136.786 s |
| Fanouts passes | 49 | 41 |
| Fanouts numeric searches | 44,772 | 31,654 |
| Fanouts active tasks at handoff | 344 | 404 |
| All tasks entering destination Moving | 591 | 621 |
| Fanouts bridge preemptions | 678 | 587 |
| Destination Moving time | 1.910 s | 0.725 s |
| Completed PNR runs | 0 | 0 |

Neither routing run timed out. Both failed in destination Moving, on different
tasks. Its shorter runtime is therefore not a quality improvement. Fanouts is
42% faster in this diagnostic comparison, but leaves 30 more tasks for Moving.
All 587 audited Fanouts preemptions committed and completed their requester;
there were no post-cut commit failures, historical cycles, immediate reversals,
or bad releases. This audit concerns replacement validity, not eventual
completion of the displaced routes.

The archived handoff snapshots and `detour_audit*.json` files in the respective
run directories show a regression in retained detour length:

| Unfinished bindings with private inter-Tile hops | Before | After |
| --- | ---: | ---: |
| Number of such bindings | 545 | 581 |
| Total private inter-Tile hops | 15,667 | 29,095 |
| Median private hops per binding | 9 | 30 |
| Longest private suffix | 226 | 349 |
| Maximum visits to one Tile on the whole retained path | 8 | 2 |
| Bindings exceeding two visits | 124 | 0 |

These count route fragments per binding, not unique leased resources; shared
fragments are excluded from private hop totals. The visit limit works, but does
not control wandering through many different Tiles. Two previously sampled
routes now complete (the 79-hop retry-loop example and the 189-hop example),
while the previous 158-hop example remains unfinished with 191 private hops.
It would be incorrect to describe the detour problem as solved.

The new failing sink is `$abc$157062$lut$aiger157061$4195/I4` (LUT5), originally
at `(30,28)/11`, driven by `$auto$abc_ops_reintegrate.cc:655:reintegrate$157298/O`.
Moving exposes five anchors, reaches ten states, and tests 17 resource-Tile
candidates. Ten have no packing resources; seven reject packing. Across their
56 LUT-slot attempts, 53 slots are occupied and three fail additional checks:

- `(14,31)/143` and `(24,32)/143`: input demand 32 exceeds limit 31.
- `(14,28)/135`: no shared lane to chain sink `$scalepnr_passthrough$1806`.

The final reached CB `(22,31)/DST3845` has four outgoing SRC choices, all busy.
Candidate congestion audits report no unowned leases or missing/stale ownership
registrations. The recorded reasons explain this search's failure; they do not
prove that every packing restriction is necessary or that the design is
unroutable. Reports and images are in this replay's
`routing_failure_candidates_001/` directory.

Conclusion: search work and repeat-visit correctness improved, but routing
completion did not, and retained partial-route lengths worsened. The next
investigation should track why these long private prefixes remain committed
while failing to approach their sinks, rather than increase the timeout.
