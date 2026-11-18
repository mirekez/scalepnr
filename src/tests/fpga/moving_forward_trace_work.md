# moving_forward_trace_work

Measure recursive work on converging branches with physical dead ends.

- Construct five layers with four alternatives at each layer.
- Vary numeric arrival IDs across ten deterministic seeds.
- Run each seed with and without internal LOCAL relays.
- Use terminal JOINT leaves instead of usable terminal LOCALs.
- Placement rejection alone must not prove a global routing deadend.
- A fresh accepting callback follows one branch to the final layer.
- Count one recursive frame per physical hop plus the root frame.
- A separate four-expansion attempt must stop at its explicit budget.
- The budgeted attempt must not persist exhausted-source marks.
- Reject all placements in the subsequent exhaustive attempt.
- Each genuinely exhausted SRC must be tried only once.
- JOINT leaves are examined inside their Tile frame.
- LOCAL relays must not add frames or restart the exit cursor.
- Check each reachable arrival receives its expected probes.
- Routing without diagnostics must not copy CB snapshots.
- Compare all live occupancy masks after the trace.
- An additional graph returns to a Tile through another DST.
- The later branch must reuse a SRC after the earlier lease unwinds.
- Any busy SRC may become free and cannot prove a persistent deadend.
- A fresh attempt must also retain the usable continuation.

History:

- Clarified physical exhaustion separately from rejected terminal placement.
Exhaustion uses transit-only Tiles and must not call placement probes.
History: changed work counts to physical CB hops; eligibility stays separate.
