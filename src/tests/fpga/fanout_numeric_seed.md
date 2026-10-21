# A repaired numeric trunk disappears at the next Fanouts handoff

Example 1 completed run 4, then stopped in run 5 with `Fanout missing Generic
trunk`. This was a scheduler/representation error, not an exhausted CB search.
The affected source had a complete retained binding to its first sink, while
a second sink was deferred until Fanouts. The first binding had been repaired
to this entirely local physical path at `(54,29)`:

`LOCAL163 -> JOINT22 -> JOINT8 -> JOINT23 -> LOCAL1`

Its six stored fragments were an output PIN, four `WIRE_ROUTE_EDGE` fragments,
and an input PIN. History recorded `complete=1`, `state=routed`. The old
`bindingHasRoutedSourceExit()` additionally required a `WIRE_CROSSBAR` fragment,
so it rejected this valid seed. There was no gap to promote for repair, and
the scheduler aborted before attempting the waiting sink at `(36,12)/LOCAL22`.
The independent input-preserving audit found a free seven-jump path to that
sink after 386,216 states, without exhausting its diagnostic bound. The twelve
CB audits in the terminal/predecessor report had no ownership inconsistencies.

The regression translates the seed CB to `(0,1)` but retains the four numeric
arcs and their IDs, using abstract node names. The numeric routing committer
creates and registers the seed; tests do not fabricate busy bits. Unrelated
Basic work precedes the Fanouts handoff, as in the real incremental run.
The real generated adapter and register are represented by ordinary fixed
logic cells; the representation bug does not depend on their primitive types.

- `fanout_numeric_seed` checks seed recognition and the subsequent full flow.
- `fanout_numeric_seed_local` reaches the full scheduler barrier directly;
  before the fix it reproduces the exact missing-trunk failure.
- `fanout_numeric_seed_control` uses a conventional multi-Tile CROSSBAR seed
  and passes before the fix.
- `fanout_numeric_seed_blocked` removes the waiting sink's only physical jump;
  after the fix it fails in route search, not at the seed-recognition barrier.
- `fanout_numeric_seed_arrival` puts the three-JOINT chain before a physical
  jump. The committer emits a numeric SRC-to-DST arrival; ordinary Fanouts must
  index that arrival and branch from it, not merely succeed via a source-wide
  fallback.

All positive cases retain the completed seed, keep endpoints fixed, complete
the waiting branch, and audit every Tile's leases and registrations. The
disconnected control retains the original trunk without speculative suffixes.

The fix distinguishes **presence of physical routing** from a legacy
CROSSBAR-fragment count. Completed numeric seeds count as routed; their source
LOCAL and DST arrivals participate in the same branch index as conventional
fragments. Shared terminal attachment and source-relocation checks use the
same representation-independent predicate. CROSSBAR/hop statistics are not
redefined, and no missing or incomplete trunk is accepted merely because it
has an endpoint PIN.
