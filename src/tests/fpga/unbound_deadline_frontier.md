# unbound_deadline_frontier

Captured deadline-region free completion.

- Load the numeric free frontier recorded near the routing deadline.
- The test itself must not depend on a wall-clock deadline.
- Use the captured source and sink with abstract numeric node names.
- Install a real foreign owner on another destination entry.
- Its route must remain unchanged throughout the source trace.
- Immediate cancellation must allocate no binding or route fragment.
- Use the production sequence of explicit source budgets.
- Bounded failure must release every lease owned by that attempt.
- Proved deadends remain marked after the attempt returns.
- Check registrations independently from persistent deadend state.
- The free witness must finish without a transit cut.
- Require one complete driver-to-sink route.
- Its endpoint identities must match the original request.
- Audit all occupied nodes against registered route owners.
- Requester teardown must preserve the foreign landing lease.
- Remove the foreign fixture route only after that check.

History:

- Updated the existing case for recursive routing.
- Preserve immediate deadends independently of route leases.

Implementation:

- Exercise the production routing entry points directly.
Fixture connectivity and node names are vendor-agnostic.
