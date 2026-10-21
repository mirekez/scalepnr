# Unequal partial cuts lose a retained fork's DST

Example 1 replay `gap_162184_audit.OvwGeWo9` fails in run 4, Fanouts,
on binding 25 of canonical net 162184. Its final gap is
`(57,33)/DST66 -> (53,35)/DST26`. Repair correctly refuses the right
boundary: the congestion audit reports `CONFLICTING_DRIVERS`, including
canonical nets 162184 and 162316.

The first corrupting operation is history event 1535,
`unrouteNetRoutesFromNodes`, not the final repair:

1. Binding 4 owns the transit through `(53,35)/DST26`.
2. Binding 25 shares its incoming hop and branches through a private FORK
   that borrows DST26 (`owns_dst=false`).
3. The cut removes binding 4 through its terminal Grounding, retaining only
   the sink input. Binding 25's cut ends earlier, at `(53,34)/DST3074`.
   It keeps the next hop into DST26 and its private fork.
4. Only that first retained hop is promoted. Its landing is not owned,
   and the following FORK still borrows from the removed binding.
   At history event 1536, DST26's lease is clear although the tail survives.
5. A later cut extends the gap to DST26. Another driver has been allowed to
   occupy it; the final boundary validation then detects incompatible owners.

The regression imports the recorded 11/14-fragment parent/sibling geometry,
including the same-Tile jump, multi-Tile jumps, FORK and two-JOINT Grounding.
Upstream sibling ownership is normalized to one small scaffold trunk;
unrelated routes and congestion are omitted. Every node, Tile, cell and net
name is abstract. Two cases exercise opposite binding insertion order.

Both fail before the fix with:

```
BUG: unequal partial cuts released retained fork's borrowed DST
```

The test also checks that preview excludes the retained DST, the requested
SRCs are freed, the exact two cut extents are preserved, and all Tile leases
and registrations remain consistent. It repeats the later gap extension,
repairs through an explicitly synthetic bypass, and tears down each branch
without losing the survivor or leaking the last DST. This bypass is a test
control, not a claimed route discovered in the real device.

The fix promotes every fragment of each retained tail before any release.
This is linear in retained tail length, without new caches or searches.

Validation: both new cases fail before the production change and pass after
it (`/tmp/cut-island-red.log`, `/tmp/cut-island-green.log`). The rebuilt full
suite passes **170/170**, including all eleven puzzles, in 2217.96 seconds
(`/tmp/cut-island-all.log`, `/tmp/cut-island-all.xml`). The 50K fanout forest
passes in 256.95 seconds against its unchanged 300-second limit. Example 1
is replayed separately; these results do not claim that the full design routes.

Real replay `cut_island_fixed.R8DidDnf`: the formerly failing binding 25
completes as a 15-fragment route. Runs 4 and 5 finish in 176.352 and 274.426
seconds of routing, respectively, bringing the completed design to 4,699
cells. Run 6 then fails in Basic routing on a different net, canonical 165423.
That new failure has neither deadline expiry nor stagnation; its 13 reported
sink-area Tile audits have no occupancy/registration/driver conflicts.
It requires a separate reachability investigation.
