# Long-jump starvation during interrupted fanout repair

Example 1 run 5 stopped repairing a binding from `(136,96)/LOCAL150` to
`(11,19)/LOCAL119`. Preemption had left a gap from `(57,35)/DST3810` to
`(49,30)/DST3539`. The retained tail contained eleven Tile hops. The failed
Tile and thirteen terminal/predecessor Tile audits found no orphan leases,
missing leases, or stale registrations. An independent input-preserving
reachability audit found a complete 36-hop path without foreign preemption.

The numeric search queued `(136,84)/SRC3073` as state 55. Its physical jump
lands at `(126,84)/DST3078`, ten Tiles closer to the destination. However,
the old priority used the SRC's **departure** coordinates. This gave that
exit the same priority as short exits from the same DST. Shorter branches
were expanded first and recursively consumed the frontier. When state 55
was finally expanded, its free landing was rejected by the 32,768-state
limit. The search had allocated 32,768 states for 15,463 distinct nodes;
12,820 otherwise acceptable additions encountered the limit.

## Reproduction and its limits

`fanout_source_priority_frontier.txt` contains 62,258 distinct physical arcs:
all eligible edges logged during the failed free search (including those
discarded by dominance or repetition checks), plus the independent witness.
Rows are `from_x from_y from_kind from_id to_x to_y to_kind to_id`.
Kinds are LOCAL=0, JOINT=1, SRC=3, DST=4.
`fanout_source_priority_route.txt` contains the interrupted 36-fragment
binding: output pin, 21-hop prefix, gap, eleven-hop tail, terminal JOINTs,
and input pin. The fixture creates abstract names from numeric IDs.

This is a **reduced topology**, not an exact copy of the full chip state.
Other bindings and unrelated owner routes are omitted. Borrowed prefix
resources become primarily owned by the retained binding; endpoint resource
Tiles are colocated with their routing CBs. Removing unrelated physical
continuations also allows additional LOCAL-leaf pruning. Consequently the
old search finishes this fixture at 24,499 states rather than exhausting
32,768 as in production. The regression explicitly uses a 16,384-state bound
to reproduce the same starvation mechanism, with a 32,768-state positive
control. The production bound remains 32,768. No fabricated occupied bits,
dummy completed seed route, or increased production limit is used.

Before the priority change, the main case fails, the larger-budget control
passes, and the disconnected-terminal control correctly refuses repair.
After the change, the main case completes the 36-hop path in 97 states,
joining the same retained tail. The fix ranks a SRC using its resolved
physical landing and the existing jump cost. SRC-to-JOINT alternatives
remain represented. This changes search order only: occupancy, same-source
incoming-arc checks, repetition limits, cancellation, preemption policy, and
whole-path validation remain in force.

All cases audit ownership and registrations. Positive cases require a
complete binding, unchanged fixed endpoints, the existing hop limit, and
clean teardown. Cancellation and a removed terminal connection must not
alter the retained route. Real Example 1 must still be rerun after the full
suite; passing this reduced fixture alone does not establish a chip-level fix.
