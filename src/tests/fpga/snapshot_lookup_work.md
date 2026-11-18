# snapshot_lookup_work

Keep diagnostic snapshot lookup proportional to reached recursive frames.

- Build a 160-Tile chain with numeric routing nodes.
- Alternate direct, LOCAL and JOINT transit forms.
- Add a preferred side branch that returns to an earlier Tile.
- Preserve the original root DST lease.
- Capture each Tile's initial CB state.
- Observe the diagnostic snapshot view during placement probing.
- Require one original snapshot per Tile despite re-entry.
- Check every saved occupancy mask against the initial state.
- The successful mode must reach the final terminal.
- Check every returned physical hop in source-to-sink order.
- The discarded side branch must not remain in the returned path.
- The exhaustion mode uses JOINT leaves without outgoing arcs.
- A free terminal rejected by placement would not prove exhaustion.
- The limited mode stops at its explicit expansion budget.
- The exception mode throws from the placement callback.
- All modes must unwind only their own leases.
- Persistent deadends must never be cleared during unwinding.
- Check SRC, DST, JOINT and LOCAL occupancy after each attempt.
- Bound snapshot lookups by reached frames, not elapsed time.
- Keep source connectivity and node annotations abstract.

History:

- Restored observer calls before probes and exposed unexpected test failures.

Fixture connectivity and node names are vendor-agnostic.
