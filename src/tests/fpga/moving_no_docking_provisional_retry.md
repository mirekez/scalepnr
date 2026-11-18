# Moving without a docking neighborhood

Reject a disconnected source trace without repeating it for preemption.
Use an abstract three-Tile corridor ending before the target Tile.
Keep a mapped but disconnected input at the fixed destination.
Run with Moving and endpoint preemption enabled.

## Distant target

Place the target beyond the bounded Grounding neighborhood.
Require exactly one source-exit tracing phase.
Require at least three expansions to exercise the physical corridor.
The exitless final Tile may reject its unused LOCAL immediately.
The source phase must expand no more than the five available frames.
Total connection expansions must equal that single source phase.
Do not start a provisional docking trace without a reachable neighborhood.

## Nearby target

Move the disconnected target into the bounded docking neighborhood.
The first source trace must make a docking call.
The provisional docking phase must also exercise its bounded probe.
The absent physical connection must still prevent completion.
No post-cut source trace may run because no cut was accepted.
Preserve the original leases and endpoint placement on failure.

## History

Adjusted work accounting for early rejection of an exitless final Tile.
Retained the exact single-attempt and absent-cut requirements.
