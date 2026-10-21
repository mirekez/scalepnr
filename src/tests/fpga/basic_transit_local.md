# Basic misses internal LOCAL transitions

Example 1 completed runs 1–5, then failed in run 6 Basic while routing a
register Q to a LUT input. The destination routing input was `(16,6)/LOCAL114`.
Its 64 terminal alternatives included one free direct entry through DST242.
All 13 destination/predecessor Tile ownership audits were clean, as was the
reported failed-state Tile. The failure was neither timeout nor stagnation.

A whole-source, read-only audit found a completely free ten-hop path from
`(50,4)/LOCAL145`, reaching the sink after 343899 numeric states, without
hitting its limit. Two internal connections are essential to that witness:

- `(31,7)/DST3376 -> LOCAL196 -> SRC3539`.
- `(25,3)/DST3539 -> LOCAL177 -> SRC162`.

Both are outside the sink's immediate Docking area. Basic's fast hop search
cannot express these internal LOCAL bridges. The numeric completion used by
Fanouts already can, but Basic did not call it. The fast search instead used
200-state budgets for starting exits, wandered as deep as 196 hops, then
failed without committing the speculative route.

`basic_transit_local.cpp` imports the whole ten-hop free witness with abstract
names, preserving coordinates, numeric arcs and both internal bridges. It
omits unrelated congestion and colocates endpoint resources with their CBs.
The driver remains a register and the sink a LUT, both fixed. The test calls
the production Basic task entry, not Fanouts or a test-side router.

The direct-arc control replaces only the two bridges with direct DST-to-SRC
arcs. The disconnected control removes the second bridge's input arc. Positive cases
must complete without preemption; negative failure must retain no speculative
binding. Ownership, exact witness hops, bridge leases and teardown are checked.

Before the fix the witness test fails while both controls pass. The fix shares
the existing bounded numeric completion between Basic and Fanouts, invoked
only after their fast searches fail. It keeps the 32768-state/200-hop limits,
free-first policy, packed reservations and atomic full-path commit unchanged.
No new search engine, persistent cache, partial route or stage retry is added.
