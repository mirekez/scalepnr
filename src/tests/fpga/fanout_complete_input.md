# A retained gap misses an already completed physical input

Example 1 run 5 failed a fanout of signal 278 to sink 5801. The retained
21-hop prefix stopped at `(11,15)/DST3619`, followed by a GAP to
`(12,18)/LOCAL40`. Two siblings, sinks 5827 and 5819, were already complete
at that same input. Their terminal path was:

`(12,18)/DST1027 -> JOINT58 -> JOINT43 -> JOINT63 -> LOCAL40`.

The completed sibling's source is `(50,8)/LOCAL154`. Its full path has 18
inter-Tile hops and two distinct arrivals at `(5,19)`. The destination's
13 final Tile audits were clean; the selected-task replay likewise found
no orphan leases, missing leases/registrations, stale registrations, or
conflicting drivers in its recorded snapshots.

This is not proof based only on free destination approaches. The independent
input-preserving audit reached the sink through the exact completed sibling
path after discovering 1,173,770 states, without hitting its diagnostic
limit. Every occupied node on that witness had same-source owners.

The production source search allocated 32,768 states, expanded 29,814, and
rejected 3,595 insertions at its budget. It had queued the sibling's first
exit `(50,8)/SRC545` as state 4, but expanded it only after exhaustion:
its allowed first landing `(53,9)/DST545` was budget-rejected at cost 6,
hop 1. Increasing the budget is unnecessary when the complete path is
already registered.

The ordinary Fanout stage already supports exact-input terminal reuse.
However, a task with a retained GAP enters `repairTaskGaps()` before that
ordinary path. The correction checks live input owners there, requires a
complete same-source route with the same physical source and sink LOCALs,
and uses the existing shared-terminal attachment. Attachment registers the
replacement before freeing its abandoned private prefix. No preemption,
new fabric search, persistent cache, or search-limit change is needed.

## Reproduction

`fanout_complete_input_frontier.txt` contains 67,870 unique allowed arcs
recorded during the failed search plus 78 missing arcs from the completed
and partial paths. Rows are `x y kind id next_x next_y next_kind next_id`;
kinds are LOCAL=0, JOINT=1, SRC=3, DST=4.
`fanout_complete_input_paths.txt` records both exact numeric paths.
The final partial-path edge is a GAP, **not** a physical topology arc.
The fixture also restores the 16 raw DST-to-JOINT63 terminal arcs from the
backward report. They are absent from the allowed frontier because the
completed sibling already owns JOINT63 with a different input. Keeping this
raw topology is essential for the ordinary endpoint-mapping checks; the
ownership checks still forbid shorting those alternate inputs together.

All textual node names are generated abstractly. CB base-type IDs are
normalized to zero; resource Tiles are colocated with CBs, and the three
register slots are normalized to 0, 1, 2. The real sink resource was `(13,18)`.
Other foreign-owner routes are omitted, along with their unavailable arcs.
This is a reduced frontier, not the full device. The two complete siblings
and failed prefix have registered owners; no artificial busy bits are used.
The source LOCAL remains an unleased source reference, as in the capture.

- Main case: production 32,768-state/200-hop bounds, reproduced failure.
- Control: same paths without the distracting frontier, repairs successfully.
- Different-input control: completed siblings end at LOCAL41, while the GAP
  targets LOCAL40 and has no terminal arc. Repair must fail without mutation.

After restoring the recorded raw terminal arcs and strengthening the checks,
disabling the correction still made only the main case fail (15.87 seconds
for all three; 963.888 ms in the failed gap repair). Assertions cover
cancellation, live ownership before/after repair,
completed siblings, fixed endpoints, release of the abandoned private
prefix, and final teardown. The test calls the production gap-task handler.

With the correction restored, all three tests passed in 14.75 seconds.
The reproduced gap repair itself succeeded in **0.343 ms**, versus the
baseline's failed **963.888 ms** search. Most fixture time is topology
construction and auditing. The different-input control still failed safely;
the two completed siblings retained their exact physical paths.

An additional idempotence assertion repeats the gap-handler call after
completion and requires unchanged stored-route count and history. It caught
a first implementation's unnecessary shared copy on an already complete
binding. The shortcut is consequently restricted to actual gapped routes.
The existing `fpga.fanout_gap_failure_location` regression also caught an
early cancellation return that skipped the normal boundary diagnostic.
Cancellation now disables only the shortcut and continues through the
existing failure-reporting path; that regression is unchanged.

The final correction passed all **177/177** CTest regressions, including all
11 puzzles, in 2,140.92 seconds. The separate cancellation-location check and
all three complete-input cases also passed together before that full run.

Example 1 subsequently completed run 5 with all 4,699 cumulative cells placed
and routed (256.546 seconds for that run's routing and final checks). It then
failed on a different Basic connection in run 6, before the whole design was
finished. Clearing run 5 is not a claim that Example 1 produces a bitstream.
