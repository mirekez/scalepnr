# Basic search: templates are not physical continuations

Example 1 run 6 exhausted its 32,768-state Basic numeric search from
`(53,20)/LOCAL145` to `(15,7)/LOCAL97`. Its 1,257 recorded congestion
snapshots were clean. An independent free-only, packed-input-preserving
search found a 20-inter-Tile-hop route after discovering 1,146,011 states.
No preemption or movement was required. These coordinates identify CBs;
the real sink resource was `(14,7)`.

The production search allocated 17,034 LOCAL states. Inspection of the
actual device topology found 10,790 distinct admitted LOCALs with no
intra-CB successors and no resolvable cross-Tile successors. Their raw
cross-Tile template lists were nevertheless nonempty. For example,
`(53,20)/LOCAL81` has a template expecting base type 42 one Tile west,
but that Tile has base type 30. Testing only whether the template list
is empty incorrectly treats this dead leaf as a useful continuation.

`basic_local_templates_frontier.txt` records 77,711 distinct free arcs
considered by the failed search plus nine missing arcs from the independent
witness. Each row contains `x y kind id next_x next_y next_kind next_id`;
kinds are LOCAL=0, JOINT=1, SRC=3, DST=4. The companion
`basic_local_templates_unresolved.txt` records `x y LOCAL_id template_count`
for the 10,790 unresolved LOCALs. Counts are one or four.

The fixture preserves coordinates, numeric arcs, and unresolved-template
counts, but normalizes all traversable CBs to base type 0 and unresolved
templates to base type 1 one Tile west. Endpoint resources are colocated
with their CBs. Names are abstract. This is a reduced free-edge snapshot,
not the full device, and does not claim omitted edges are occupied. It
contains no invented congestion or preemptible victims.

The tests distinguish budget loss from an impossible route:

- Normal production budget with unresolved templates: reproduces failure.
- 65,536-state control: finds a complete 19-hop alternative.
- Production budget without unresolved templates: finds that alternative.
- Removing the sole final sink arc: remains disconnected and leaves no state.

Positive cases require one complete binding, fixed endpoints, the internal
LOCAL bridge at `(53,8)/LOCAL196`, and no more hops than the independent
witness. They do not require choosing the exact BFS witness. Cancellation,
pre/post ownership audits, and teardown checks cover speculative mutation
and leaked leases.

The correction resolves cross-Tile templates at the actual
Tile when all intra-CB successor masks are empty. Source and sink are
exempt; real LOCAL bridges and resolvable dedicated links remain eligible.
Neither production budgets nor congestion policy changes.

Validation: before the production change, the normal-budget case failed
and all three controls passed (23.77 seconds total). After the change,
all four passed (39.23 seconds under concurrent unrelated host load).
These wall times are not a controlled performance comparison. Full-suite
and real-example validation are separate from this focused result.

The subsequent complete suite passed **174/174**, including all 11 puzzles,
in 2,274.13 seconds. The optimized Example 1 executable was rebuilt from
the same sources before replay; its result must be checked separately.

The replay subsequently stopped in run 5, before the original run-6 task,
on a different retained-gap fanout. It therefore does **not** establish that
Example 1 is fixed. That failure and its exact-input reuse regression are
documented in `fanout_complete_input.md`.
