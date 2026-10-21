# Free prefix missed before legal terminal preemption

Example 1's fresh shared-model replay completed runs 1–5, then failed run 6
Basic routing on signal 166890. The CB endpoints are `(41,8)/LOCAL146` and
`(37,36)/LOCAL83`. This is not the earlier signal-166890 snapshot whose source
was elsewhere. All 2,140 frontier ownership audits and 13 sink-neighborhood
audits are clean. There is no timeout, stagnation or OOM.

The normal source search allocates 32,768 states, reaching only to Chebyshev
distance six from the sink. Grounding consequently has no starting states.
An independent free audit exhausts 2,929,083 nodes without reaching the input.
Its whole free component nevertheless reaches 55 Tiles inside the Grounding
window: the bounded frontier is not the whole reachable component.

A separate read-only probe, using the actual production preemption callback,
validates a complete 38-node / 17-Tile-hop route. Its free prefix approaches
via `(25,37)/SRC1031 -> (36,37)/DST1031`. Terminal completion is:

```
(36,37)/DST1031 -> SRC262
    -> (37,36)/DST260 -> JOINT23 -> JOINT9 -> LOCAL83
```

SRC262 and DST260 belong to one completed private transit branch of a
17-binding retained net. Both are legal cuts. Its Takeoff and Grounding are
elsewhere, and the contested segment is not shared by the other bindings.
The read-only probe does not cut it or rescue the failed task.

The regression preserves 104,554 unique admitted/witness arcs, 12,271 raw SRC
landing/JOINT metadata rows, the complete witness, and the obstructing private
branch's 17 fragments. Unrelated branches are omitted; shared prefix ownership
is normalized to the isolated private route. Every label is an abstract node
name. Omitted occupied outlets have live, routed obstruction nets, not unowned
busy bits. This is a reduced captured frontier, not the complete device graph;
its state counts therefore differ from the real device's larger search.

Fixture construction must finish before resolving new jumps. In particular,
the synthetic obstruction's added landing must invalidate derived masks.
An initial fixture setup failure at that point was discarded, not reported
as reproduction of the router bug.

With unchanged priority logic, the normal-budget regression fails with
`normal numeric frontier misses a validated 17-hop Grounding approach`.
The larger-budget, protected-victim and disconnected-terminal controls pass.
The successful control atomically transfers the contested nodes and repairs
the retained victim through a private repair outlet. Every case audits real
ownership; cancellation and rejected searches must leave no new binding/cut.
The protected control uses enough states to actually inspect its owner.

An initial blanket half-bias trial passed all four captured cases and the
190-test suite (including eleven puzzles), but the real replay changed earlier
successful routes, spent more time in run 5 and late placement, then failed
on a different run-6 connection before this one. It was not end-to-end success.

The refined correction preserves every original successful attempt and its
selected victims. Only after all original attempts fail does it make one
additional bounded free-prefix/Grounding attempt with half the geometric bias.
It does not repeat unchanged Takeoff search. Per-attempt state/hop limits,
stage deadlines and preemption rules stay unchanged. The normal-budget test
requires the alternate attempt; the larger-budget control verifies that an
original success never invokes it. These cases, protected/disconnected
controls, diagnostic safety and `fpga.source_fallback_precedence` pass together.
The full 191-test suite then passed, including all eleven puzzles, with
unchanged limits (1,263.26 seconds at two workers). Fresh Example 1 validation
remains required before claiming end-to-end success.

## Stronger task-level recovery capture

The fresh original-first replay reproduced the same source/sink placement and
failure. The real half-bias attempt also exhausts 32K (32,750 unique nodes,
3,659 rejected additions) with no Grounding starting state. Cost-scale-4 and
-8 read-only controls at that bound fail too. A larger read-only probe again
validates the same 17-hop route; its free-prefix seed index is 52,930.

The first reduced graph omitted arcs visited only by alternate priority. The
`grounding_detour_recovery` case adds exactly those captured arcs and their
physical SRC metadata: totals are 122,286 arcs and 14,369 metadata rows. It
does not replace the four original cases. All labels remain abstract and all
busy nodes have real owners. It enters through `routeTaskOrFail`, not directly
through a manually enlarged low-level call.

Before the caller correction, this stronger case fails with the same Basic
failure, both normal numeric priorities exhausting 32K and Grounding states
remaining zero. That is the valid RED result (30.72 seconds); fixture setup,
free-reachability and ownership checks pass first. The caller now exhausts
all original pin-pair attempts, then retries at 65,536 states. Previously
successful attempts keep their choices; the low-level explicit budget is not
silently overridden. The new assertion requires this caller-owned recovery
and the existing assertions verify one complete replacement, one cut victim,
successful victim repair, unchanged fixed endpoints and clean teardown.

All 192 tests were then rerun. The full two-worker run passed 189; three large
routing puzzles hit deadlines while the machine was shared. The unchanged
subtype binary also slowed from 273 to 406 seconds. Serial reruns of all three
timed-out puzzles passed without source or limit changes: 50K/50% in 248.74s,
50K/90% in 256.03s and the forest in 269.84s. Thus all 192 cases pass across
the full run and isolated reruns. Real Example 1 remains separate validation.

The next real replay completed the previously failing 166890 connection with
the same 17-hop witness at seed 52,930: 20 stored fragments and one preemption.
Runs 1–5 never used recovery. Run 6 then passed several more numeric recoveries
and failed a later, different signal 165268. Thus this specific captured miss
is repaired, but the complete Example 1 flow is not yet successful.
