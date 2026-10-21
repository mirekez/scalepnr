# Local completion beyond short Grounding patterns

`fpga.fanout_long_grounding` is implemented in `fanout_gap_replay.cpp`.
All fixture node, net and cell names are synthetic.

## Captured failure

A driver and sink share CB `(45,38)`. Their completed five-hop connection was
preempted by a new Takeoff at `(45,39)/SRC65`. The old route retained seven
fragments, including its source hop, an explicit gap from `(45,39)/DST20` to
`(41,41)/DST3073`, and its destination tail ending at LOCAL88 through JOINT64
and JOINT49. Ownership audits found no orphan leases or stale registrations.

The live free-source audit exhausted 14 states, without reaching its budget.
That established only that a free replacement was absent. A separate read-only
local probe, using the actual preemption validator, found this complete path:

```
(45,38) LOCAL154 -> SRC3076 -> (41,38) DST3073
 -> SRC258 -> (44,38) DST262
 -> SRC259 -> (45,38) DST257
 -> JOINT26 -> JOINT12 -> JOINT64 -> JOINT49 -> LOCAL88
```

Only `(45,38)/SRC3076` and `(41,38)/DST3073` require preemption. Both belong to
one interrupted transit binding. The complete path and combined releases
passed validation; the probe committed nothing. The old repair nevertheless
failed because its preemption-assisted completion was restricted to short
patterns, excluding this source departure and longer terminal JOINT chain.

## Regression scope

The fixture reproduces the original middle cut, seven requester fragments,
the contested owner's 15 fragments and two earlier gaps, and the exact
three-hop/four-JOINT replacement. It omits 29 unrelated owner bindings and
unrelated blocked alternatives; consequently it is not a full-chip occupancy
replay. Shared-prefix ownership is normalized for the omitted replicas. All
retained busy resources are backed by actual route fragments and registrations.

Before the fix, the regression reaches its intended failure:
`repair misses validated three-hop Takeoff/ Grounding with four terminal JOINTs`.
After the fix it checks physical continuity, all terminal JOINT leases, exactly
one requeued transit victim, preserved victim endpoints, the untouched newer
Takeoff, and clean ownership. Disabled preemption, a protected owner, and each
broken link in the terminal JOINT chain must leave routing unchanged.

## Fix boundary

Keep existing free and short-completion searches first. If they fail, continue
over numeric topology from reached states inside the five-Tile Grounding
window, at most ten additional inter-Tile hops. Preview eligible transit
obstructions without cutting. Validate the full prefix, terminal chain and
combined releases before the existing atomic commit. Total repair hop/state
limits, cancellation, protected endpoints and packing reservations still apply.

The final-jump and two-jump hop-bound controls allow ownership previews but
assert that no commit occurs and no lease changes when the full path exceeds
the limit. A preview is not a committed preemption.

## Validation

The complete 137-test suite (including eleven puzzles) passed after this fix.
The subsequent full example replay reached a different interrupted route in
run 4 and exhausted the unchanged 600-second Fanouts budget; this is not a
claim that the whole example routes successfully. Its sink-area audit checked
2,631 leased nodes across thirteen Tiles with no unowned leases, missing
leases, missing registrations or stale registrations. Phase profiling is
needed to separate repair-search cost from the remaining preemption workload.
