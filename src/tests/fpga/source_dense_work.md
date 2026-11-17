# source_dense_work

Purpose: bound source-trace work in a densely reconvergent corridor.
Increasing parallel lanes must not require exploring every lane combination.
Implementation: `source_dense_work_regression` in `fanout_gap_replay.cpp`.
Run: `ctest --test-dir build -R '^fpga.source_dense_work$' --output-on-failure`.

The fixture contains forty-eight Tiles along a horizontal corridor.
Variants provide one, eight or thirty-two parallel routing lanes.
Every arrival connects to every forward exit in the next Tile.
All terminal arrivals feed the requested sink LOCAL.
Wide variants include a registered foreign owner of one intermediate DST.
That obstruction requires the requester to select another free lane.
Each width runs with and without detailed frontier diagnostics.
The chosen route must be identical with diagnostics enabled.

Completion must install exactly one complete binding with forty-seven jumps.
Cancellation after one hundred polls must preserve all routing state.
Owner audits check the requester and the protected foreign route.
Unrouting must empty the requester's route and release its private resources.
The foreign owner's route history must remain unchanged.
Source labels are bounded by three times the corridor length.
Eligibility checks stay proportional to the path; prefix-walk checks must be zero.
Diagnostic edge counts must cover that path without flooding unrelated lanes.
The test uses operation counts, not a machine-dependent time limit.

History: formerly named `source_dominance_work` for breadth-first traversal.
Depth-first tracing completes before unrelated lane arrivals need expansion.
Source recursion rejects its own occupied nodes without walking the prefix.
History: owner checks moved to Docking; Wire supplies numeric angle choices.
