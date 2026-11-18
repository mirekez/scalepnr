# backward_rejected_placement_preserves_src

Keep forward connectivity usable after backward placement rejection.

- Build two incoming branches that converge on one shared SRC.
- Connect that SRC to both branch arrivals with numeric mappings.
- Provide a real driver LOCAL at the shared Tile.
- Connect both branches to the requested destination.
- Build the production backward resolve index.
- Reject every proposed driver placement in the first attempt.
- Require that the attempt reports failure.
- Require that the shared SRC was actually offered to the callback.
- Require that rejection does not set its persistent deadend bit.
- The failed search explored predecessors, not outgoing continuations.
- The SRC still has a physical forward path to the destination.
- Accept the shared driver LOCAL in a second attempt.
- Require that the second attempt offers the same SRC.
- Require successful completion of that takeoff.
- Keep an unrelated LOCAL lease throughout both attempts.
- Compare SRC, DST, JOINT and LOCAL occupancy before and after.
- Compare pin leases before and after.
- Neither failure nor successful probing may leak a trace lease.
- A one-expansion control must stop explicitly without leases or deadends.
- The test uses the same graph for both placement policies.

History:

- Replaced the old expectation that backward rejection globally poisons a SRC.

Fixture connectivity and node names are vendor-agnostic.
