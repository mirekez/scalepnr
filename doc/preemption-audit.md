# Docking bridge preemption audit and correction

## Fix plan and implementation

1. Cover the blocked-terminal bridge, independently occupied replacement
   resources, rejected commit, and later-blocker reversal with regressions.
2. Treat blocked-terminal frontiers as geometry, not lease proofs. Compute the
   exact victim cut's releasable footprint, excluding retained-prefix and
   foreign-owned resources; the victim may itself own the blocked terminal.
   Prepare all replacement leases before invoking the destructive cut. Restore the original live state
   for the ownership/policy check, then publish only the prepared lease additions
   after the cut. There is no fallible lease operation after victim removal.
3. Retain every distinct blocker parent and check the complete ancestry graph
   across passes; do not overwrite or ignore older/later parents.
4. Run focused regressions, then example 1 with 500-, 1,000-, and 2,000-cell run
   targets, recording per-stage time, unfinished tasks, and preemption outcomes.
   Increase further only when the smaller run results justify it.

The changes are implemented in `RouteDesign.cpp`, `Docking.cpp`, `Wire.cpp`, and
`RoutePassState.h`. The transaction helper only copies route/footprint-touched Tile state,
not the device. Its preflight checks SRC, DST, both JOINTs, LOCAL and terminal
pin leases, including conflicts between fragments of the proposed path itself.
On failure or policy rejection no victim has been cut. On success it publishes
only newly acquired bits, so it cannot resurrect released victim-tail leases.

Validation: `fpga.routing`, `fpga.docking`, `fpga.moving`, and the separately
selectable `fpga.bridge_preemption` pass; all nine offline analyzer tests pass. The full-route atomic
commit applies to full-path Docking bridge exchanges in Basic, Fanouts, and
Moving. Moving's separate
reverse-boundary exploration and other preemption kinds retain their existing
policies; the ancestry fix applies to every caller of the common cycle guard.

The original audit below describes the **pre-fix** behavior, not current policy.

### First implementation's size sweep (before ownership-footprint refinement)

All runs use the same example 1 RTL, constraints and device, with
`SCALEPNR_ROUTE_PREEMPT_AUDIT=1`, `SCALEPNR_ROUTE_PROFILE=1`, and
`SCALEPNR_ROUTE_PASS_DETAIL=1`. Run-size targets do not split bunches, so actual
admitted cell counts differ. Routing times exclude import and placement.

| Target | First batch | Routing time | Fanouts | Later result |
| --- | ---: | ---: | --- | --- |
| 500 | 465 cells | 9.236 s | 4 passes, 323 to 0 tasks, 0 preemptions | Batch 2: Const routing failed |
| 1,000 | 727 cells | 20.561 s | 40 passes, 863 to 2 tasks; Moving completed both; 0 preemptions | Batch 2: Moving destinations failed |

500 log: `tests/prjxray/1/build/run.mhiWBbDd/scalepnr.log`. Batch 2 adds 262 cells;
its failing net is `VCC_NET.$auto$ff.cc:337:slice$17155.CE`. Its terminal at
CB `(125,11)` requires JOINT 56, owned by `$auto$bufnorm.cc:387:execute$162021`.
The saved pre-fix 500 run (`run.S2Y7m3Ap/scalepnr.log`) fails on the same batch
and net. Neither first batch required preemption, so its elapsed-time difference
is not evidence of speed gained from these preemption fixes.

1,000 log: `tests/prjxray/1/build/run.510ETfVI/scalepnr.log`. Batch 2 adds 989
cells. Its Fanouts stage takes 57.456 s / 40 passes, reducing 1,253 tasks to 69;
Moving destinations takes 4.041 s, leaving 60 tasks when it reports no candidate
for `$abc$157062$lut$aiger157061$3301` (LUT5). The cell was at `(14,34)/3`; the
reported reached packing candidate was `(13,33)`, with all eight positions
rejected. Neither batch used preemption and no stage timed out. This is a
remaining placement/search failure, not proof that the complete design is
unroutable.

### Ownership-aware refinement results

The initial implementation was too restrictive: a blocked terminal may belong
to the same victim tail and become free after that exact cut. The refined
preflight uses `previewNetRouteCut()` to include those resources, but excludes
the retained prefix and every node with another surviving owner. Regression
coverage includes both the independent-blocker rejection and same-victim
acceptance, plus a tail JOINT shared with a second binding and terminal-pin reuse.

The 1,000-target rerun is `tests/prjxray/1/build/run.4dsCBCnc/scalepnr.log`.
Its first 727-cell batch completes in 20.537 s routing time. Batch-2 Fanouts:

| Metric | Saved pre-fix `run.fdURaSbA` | Ownership-aware rerun |
| --- | ---: | ---: |
| Time, 40 passes | 29.112 s | 25.586 s |
| Tasks at handoff | 15 | 9 |
| Accepted bridge preemptions | 153 | 76 |
| Completed / partial victims | 128 / 25 | 68 / 8 |
| Removed fragments | 1,284 | 475 |

The complete bridge audit covers all 76 accepted cuts: all requesters commit and
complete, zero historical cycles/reversals, zero bad releases, zero missing
records. There are 68 neutral and 8 improving exchanges, no worsening exchange.
No timeout occurs. The later Moving destination failure remains: LUT4
`$abc$157062$lut$aiger157061$20520`, previously at `(13,30)/3`, cannot pack at the
reached candidate `(14,30)`. The design is not completely routed.
This run preceded the final explicit packed-terminal reservation guard; the
2,000-target validation uses that guard as well.

### Final 2,000-target result

Log: `tests/prjxray/1/build/run.fS7O6W5d/scalepnr.log`; complete Fanouts event
audit: `preemption_audit.json` in that directory. The first batch admits 1,989
cells. Placement takes 48.975 s. No routing stage times out.

| Routing stage | Time | Outcome |
| --- | ---: | --- |
| Clock | 14.268 s | 151 sinks routed |
| Const | 0.086 s | 231 tasks completed |
| Basic | 7.422 s | One task passed to source recovery |
| Moving sources | 0.007 s | Remaining task completed |
| Fanouts | 220.731 s | 49 passes; 2,461 to 344 active tasks |
| Moving destinations | 1.657 s | Failed to find a legal destination |

The new Fanouts stage handed off after 49 passes, so compare the old audit at
the same pass count, not its later 105-pass checkpoint:

| Fanouts metric, first 49 passes | Before (`run.gqQ2WXYe`) | After |
| --- | ---: | ---: |
| Sum of pass times | 236.982 s | 220.733 s |
| Active tasks left | 345 | 344 |
| Accepted bridge preemptions | 1,465 | 678 |
| Requester failed to commit | 603 | 0 |
| Completed victim cut without requester completion | 402 | 0 |
| Historical cycles / immediate reversals | 117 / 109 | 0 / 0 |
| Removed fragments | 12,476 | 5,814 |

All 678 new Fanouts preemptions commit and complete the requester: 549 neutral
exchanges and 129 improving exchanges. Coverage reconciles exactly with pass
counters. There are no missing records, bad releases, or duplicate-owner cuts.
Basic's two bridge exchanges and Moving's one recorded bridge exchange also
complete successfully. Moving additionally makes two takeoff preemptions outside
this bridge-specific audit, which are explicitly reported as uncovered.

The remaining failure is in Moving destinations, not a timeout or a post-cut
commit failure. The recorded relocation is from `(43,16)/3`; its last rejection
is `no distinct free terminal path` for port `I2` of route
`$abc$157062$lut$aiger157061$6595.$techmap160538$abc$157062$lut$aiger$o3565.A[3]`.
The stage report's task pool grows from 591 to 665 while rebuilding incident
work around relocations. These are not the same pool as the 344 active Fanouts
tasks: the handoff also contains 247 deferred Moving tasks.

Conclusion: the two audited acceptance defects are eliminated in the reruns,
with much less discarded routing work. At equal pass count, unfinished work
is almost unchanged. This is a correctness improvement and a modest observed
time reduction, **not** a solved placement/routing problem. The size sweep
stops at 2,000 rather than increasing the batch while it still fails.

## Result

Not every accepted preemption was justified. Two reproducible acceptance defects
exist: a joined bridge can still terminate at an independently occupied resource,
and the cycle guard discards later blocker relationships. Node release ownership
checks passed in the recorded sample. The original recording was diagnostic-only;
the corrections described above subsequently changed acceptance and commit order.

## Recorded runs and complete accounting

Example 1 used a 2,000-cell run target (1,989 cells in the first run).
The main recording was deliberately stopped after 105 Fanouts passes, with 236
tasks remaining; it is not a successful complete routing run. Its stage counters
and routing outcomes match the previous non-audit 105-pass checkpoint.

- Full trace: `tests/prjxray/1/build/run.gqQ2WXYe/scalepnr.log`.
- Every accepted cut, release, outcome and cycle witness:
  `tests/prjxray/1/build/run.gqQ2WXYe/preemption_audit.json`.
- Detailed five-pass replay:
  `tests/prjxray/1/build/run.BwBZNW3a/scalepnr.log` and
  `preemption_audit.json` in that directory.

| Check, first 105 Fanouts passes | Count |
| --- | ---: |
| Accepted bridge preemptions | 2,189 |
| Completed / partial victims | 1,638 / 551 |
| Removed route fragments | 19,936 |
| Requester completed in that search | 1,352 |
| Requester did not commit | 837 |
| Completed victim cut without requester completion | 576 |
| Partial victim cut and requester completed | 290 |
| Neutral exchanges by completed-connection count | 1,323 |
| Accepted historical cycles | 406 |
| Of these, immediate reversals of the latest blocker | 333 |
| Later blocker relationships ignored by the router | 989 |
| Missing outcomes or release records | 0 |
| Released node still occupied or owned | 0 |
| Multiple-owner cuts / repeated victim in the same pass | 0 / 0 |
| Difference from independent stage preemption counters | 0 |

All recorded bridges were joined. Unjoined-frontier preemption is therefore not
the explanation for these observed failures. There were no takeoff or terminal
preemptions in this Fanouts segment outside the bridge audit.

A search can fail and a subsequent attempt in the same task can succeed. Counting
the final bound-route outcome once per task avoids overstating that damage:
2,150 tasks made the 2,189 cuts; 291 reduced unfinished connections, 1,313 were
neutral, and 546 increased unfinished connections. The net immediate change
across these preempting tasks was **272 additional unfinished connections**.
This is event/task accounting, not a prediction of a run with preemption disabled.
A neutral exchange is not proof of useful progress, especially when reversed.

## Defect 1: joined does not mean one removable obstacle

In `src/pnr/route/Docking.cpp`, `finish_frontiers()` appends bridge candidates
from the normal backward search. After that search fails, Docking also searches
backward from **blocked terminal seeds** and calls `finish_frontiers()` again.
Candidates from that second search can have `joins_frontiers=true` while their
terminal remains occupied independently of the bridge victim.

`materializeDockingBridge()` validates fragment geometry and Tile visits; it
does not prove that every resource can be leased. In `RouteDesign.cpp`,
`preemptDockingBridge()` permanently cuts/requeues the victim before
materialization and before `lease_docking_fragments()` checks the full path.
The latter can fail at the terminal. Its rollback restores the requester's
temporary leases, not the already-cut victim.

The detailed first-five-pass replay captured 411 preemptions. All 411 proposed
bridges materialized, but 164 commits failed afterward: **163 terminal lease
failures and one intermediate SRC-busy failure**. Thus geometric materialization
success is demonstrably insufficient for destructive acceptance.

### Actual failure: event 7, Fanouts pass 1

- Requester:
  `$abc$157062$lut$aiger157061$3244.$techmap158231$abc$157062$lut$aiger157061$3379.A[2]`.
- Victim: `$auto$ff.cc:337:slice$30584.$abc$157062$aiger$o58[0]`.
- Accepted cut: `(12,16)/SRC:18`; completed victim truncated from 21 fragments to 7.
- Bridge joined and materialized, but the requester did not commit.
- Failure: terminal fragment 4 at `(12,17)`, `DST:21`, `LOCAL:96`, `JOINT:49`.
- The recorded state has DST and LOCAL free, but **JOINT:49 occupied**;
  `pin_leased=false` does not establish that the complete terminal path is free.

Event 212, pass 3, demonstrates the other failure class: after a cut at
`(21,9)/SRC:3617` (victim 30 to 22 fragments), commitment fails at
`(16,12)/SRC:20` with `reason=src_busy`.

### Independent four-Tile reproducer

A small probe using the real `dockGrounding()` and `materializeDockingBridge()`
functions creates a straight path `(0,0)` to `(3,0)`, with a busy bridge SRC at
`(2,0)` and an independently busy terminal DST at `(3,0)`. Results:

```text
route_success=0 bridges=1 terminal_dst_busy=1
bridge=(2,0)/SRC:256 joined=1 materialized=1 fragments=5
after_bridge_cut route_success=0 terminal_dst_busy=1
after_both_free route_success=1
```

This establishes the defect independently of the large design's congestion.
The probe was diagnostic, not a regression asserting that this incorrect
acceptance behavior must be preserved.

## Defect 2: first-blocker-only ancestry permits reversals

`rememberPreemptionBlocker()` in `src/pnr/route/RoutePassState.h` uses
`try_emplace(victim, blocker)`. `preemptionWouldCycle()` follows that one saved
parent. Retaining the first relationship does not retain later relationships:

```text
A preempts B: record B -> A
C preempts B: discard B -> C because B already has a parent
B preempts C: guard cannot see the discarded relationship; reversal accepted
```

A direct probe of these production helpers confirms that final reversal is
allowed. The full trace independently contains this same-node example:

- A = `$abc$157062$lut$aiger157061$13887.rv32m_inst.bext[12]`.
- B = `$abc$157062$lut$aiger157061$13641.rv32m_inst.bext[9]`.
- Event 530, pass 7: A cuts B at `(11,11)/SRC:66`, victim 11 to 8 fragments;
  A completes. B's existing first blocker prevents recording A.
- Event 623, pass 9: B cuts A at **the same `(11,11)/SRC:66`**, victim 37 to 34
  fragments; B completes. The offline audit reports event 530 as the reversal
  witness. Both are neutral exchanges, not two units of net progress.

## Corrections identified by the original audit

1. Distinguish free-terminal bridge candidates from blocked-terminal diagnostic
   candidates. Joining their frontiers does not make the latter sole-obstacle
   completion opportunities.
2. Validate the complete proposed resource claim before cutting a victim, and
   preserve the victim if the subsequent commit fails. Checking just the bridge
   resource or just materialized geometry is insufficient.
3. Preserve all blocker relationships needed by cycle detection, or implement
   an equivalent sound guard; retaining only the first blocker is insufficient.

These findings do not justify banning all completed-route preemption: some
exchanges may be necessary. They identify specific unjustified cuts and cycles.

## Repeating the audit

Enable `SCALEPNR_ROUTE_PREEMPT_AUDIT=1` for the routing process. This adds numeric
cut/release records, post-cut commit diagnostics and per-search/per-task outcomes.
It does not turn on verbose logging for every failed search before a cut.

```sh
python3 tests/prjxray/preemption_audit.py LOG --stage Fanouts --through-pass 105 --json
python3 -m unittest discover -s tests/prjxray -p preemption_audit_test.py
ctest --test-dir build -R '^fpga.(routing|docking|moving|bridge_preemption)$' --output-on-failure
```

The pass filter excludes records from an incomplete following pass. The offline
analyzer checks full recorded blocker history independently of the router's
first-parent guard and reconciles coverage against stage counters. Other
preemption kinds are reported as uncovered if present; this is not a claim to
have audited every kind or every routing stage.
