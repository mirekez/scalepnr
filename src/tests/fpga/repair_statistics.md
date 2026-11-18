# repair_statistics
Purpose: verify repair profiling without changing routing behavior.
Reuse the retained-GAP source completion policy fixture.
Run the same four repairs with profiling disabled and enabled.
Two corridor lengths exercise different amounts of routing work.
Each length includes an allowed-cut and a refused-cut case.
Disabled profiling must leave repair call counters at zero.
An unrelated net filter must suppress both GAP and source counters.
A binding-name filter must select the intended connection.
Filtering must not depend only on the logical net name.
Count eight GAP calls across the four selected cases.
Count four immediately cancelled calls.
Count four real free source searches.
Do not report any reused cached frontiers.
Count two successful GAP completions.
Count two successful source completions.
Success counts must agree with installed complete routes.
The straight corridors must not require repeated node expansions.
Count common nodes visited by GAP and source searches.
Overlap cannot exceed either search's unique node count.
The existing GAP walker reports its parent reconstruction work.
Recursive source leasing must report zero parent-chain lease checks.
Preserve the fixture's complete-path assertions while profiling.
Preserve cancellation callback identity and invocation counts.
Preserve owner audits and refused-cut behavior.
Print totals using the documented parseable repair record format.
The record must identify its supplied context and GAP call count.
Resetting profiling must clear accumulated counters.
History: aligned counters with one uncached source search per repair.
History: retained profiling-off, net-filter, and binding-filter controls.
