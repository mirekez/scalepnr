# moving_whole_task_current_placement_fixed

Reject an unavailable current sink before searching free placements.

- Install an ordinary source tree and an unfinished sink binding.
- Use registered foreign routing to obstruct the current connection.
- Audit the fixture so occupied nodes have actual route owners.
- Keep a free placement reachable from the source tree.
- Run the destination mover through this case's entry point.
- Preserve the selected fixed or movable placement policy.
- Record the failure-decision history.
- Locate the beginning of free placement scanning.
- Check that the current placement is considered before that scan.
- Its unavailable mapped terminal must produce an explicit rejection.
- Do not recursively search toward a terminal that cannot be leased.
- For whole-task entry, check that routeNetTask runs once.
- Do not repeat the rejected current-placement attempt.
- For direct entry, require the same terminal eligibility check.
- Require the expected relocation or takeoff result.
- Keep completed sibling routing intact.
- Check preemption counts against the fixture policy.
- Preserve the caller's preemption setting after probing.
- Validate affected bindings and their endpoint identities.
- Retain ownership and lease audits after the operation.

History:

- Check the early terminal rejection instead of requiring an unnecessary trace.

Fixture connectivity and node names are vendor-agnostic.
