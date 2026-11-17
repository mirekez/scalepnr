# fanout_gap_probe_budget

Purpose: repair a GAP with wide gap and source frontiers.
The complete replacement must find the free one-hop source bypass.
Implementation: `fanout_gap_budget_regression(false, true)`.
Run: `ctest --test-dir build -R '^fpga.fanout_gap_probe_budget$' --output-on-failure`.

The fixture shares the retained route shape with `fanout_gap_budget`.
A transit cut leaves a GAP and a disconnected sink-side island.
A registered foreign Takeoff owner holds the cut resource.
The gap frontier contains 2,000 additional physical exits.
Those exits land on LOCAL branches without a sink continuation.
The source additionally has 1,398 exits toward a separate Tile.
SRC3500 provides the useful source-to-sink bypass.
All node names and connectivity are abstract synthetic test data.

Success requires a complete one-hop route to the sink LOCAL.
The sink pin must remain leased to the repaired binding.
The foreign owner's route and history must remain unchanged.
Owner audits reject leaked leases or missing route ownership.
Immediate cancellation must leave the retained route unchanged.
One-state and zero-hop controls must respect the caller's limits.
A disconnected-sink control stops at its 2,500-poll cancellation bound.
Success allows 8 * (2,000 + 1,398 + 16) = 27,312 polls.
This allows branch traversal, backtracking and source fallback work.
The observed successful call uses 4,011 cancellation polls.

History: the name originally described an early source-probe size limit.
The current repair traversal reaches source fallback after gap exploration.
The test keeps its wide fixture and ownership checks with a matching budget.
