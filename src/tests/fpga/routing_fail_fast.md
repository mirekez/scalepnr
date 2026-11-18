# routing_fail_fast

Reject impossible destination terminals before transit.

- Build a synthetic fabric with many free transit alternatives.
- Occupy every mapped terminal LOCAL for the first task.
- Provide a second task with a reachable sink.
- Exercise Generic, Fanouts, Moving, and constant dispatch policies.
- Soft failure must hand off the first failed task immediately.
- Hard failure must identify that task and throw the expected error.
- The later routable task must not be attempted.
- No transit edge can make an occupied terminal available.
- Check terminal availability before entering recursive generic tracing.
- Require no newly accepted transit edges.
- Leave all previously installed leases unchanged.
- Do not store an incomplete suffix or falsely retire the task.
- Write the failure report and a valid visualization.
- Keep the bounded decision history in chronological order.
- Reports must name the failed binding and correct failure policy.
- Release all task-local routing scopes after failure.

History:

- Revised the existing regression for recursive routing and immediate deadends.

Coverage:

- Uses production entry points and explicit ownership assertions.

Fixture connectivity and node names are vendor-agnostic.
