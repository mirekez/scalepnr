# route_region_replay

Captured-region repair and victim accounting.

- Load the abstract captured routing region and all live bindings.
- Keep the recorded endpoints, masks, and physical connectivity.
- The captured witness proves a replacement is physically possible.
- Check every witness arc before asking the router to repair.
- Audit the initial resources against their registered owners.
- An expired stage must neither expand nor modify routes.
- Resume repair with an explicit 131072-label budget.
- Require a complete source-to-sink route within that budget.
- The selected route may differ from the independent witness.
- Record every physical binding before and after repair.
- Detect actual displaced bindings from changed physical fragments.
- Each displaced binding must retain an explicit repairable GAP.
- The pending task set must exactly equal the displaced binding set.
- No physical binding may be queued twice.
- Preemption counters must match the actual displaced owners.
- Keep endpoints fixed and finish with a whole-region ownership audit.

History:

- Revised the existing regression for recursive routing and immediate deadends.

Coverage:

- Uses production entry points and explicit ownership assertions.

Fixture connectivity and node names are vendor-agnostic.
