# fanout_gap_budget

Purpose: repair a GAP after exhausting a broad, unusable gap frontier.
The complete replacement must use the free one-hop source-to-sink bypass.
Implementation: `fanout_gap_budget_regression(false)` in `fanout_gap_replay.cpp`.
Run: `ctest --test-dir build -R '^fpga.fanout_gap_budget$' --output-on-failure`.

The initial route has fifteen fragments and a two-JOINT Grounding.
A real transit cut leaves fourteen fragments, including the GAP.
The GAP starts at (16,8)/DST19 and retains a disconnected sink-side island.
A foreign Takeoff owner occupies the original cut resource.
There are 256 additional gap exits with resolved physical landings.
Their destination LOCALs have no continuation to the sink.
A separate exit from (11,6)/LOCAL149 reaches (12,7)/LOCAL107.
All node names and connectivity are abstract synthetic test data.

Success requires a complete one-hop replacement and leased sink pin.
The foreign owner's route and history must remain unchanged.
The fixture audits occupied resources against registered route owners.
Immediate cancellation must leave the retained route unchanged.
One-state and zero-hop controls must respect the caller's limits.
A disconnected-sink control must stop at its 200-poll cancellation bound.
The successful call allows 8 * (256 + 16) = 2,176 cancellation polls.
This covers branch traversal, backtracking and source fallback work.
Polls count checks during traversal, not distinct fabric nodes.
The observed successful call uses 523 polls with the current fixture.

History: the original test required an early source probe within 200 polls.
The current depth-first traversal exhausts the gap choices before fallback.
The test budget now covers that work without changing routing choice order.
