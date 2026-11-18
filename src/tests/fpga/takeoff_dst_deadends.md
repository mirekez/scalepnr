# takeoff_dst_deadends

Purpose: require full reverse Takeoff to filter DST rather than SRC marks.
Run: ctest --test-dir build -R "^fpga.takeoff_dst_deadends$" --output-on-failure.
Implementation: takeoff_dst_deadends_regression in fanout_gap_replay.cpp.

The fixture uses only abstract numeric routing nodes.
It contains a source Tile, an intermediate Tile, and a sink Tile.
Physical jumps connect SRC10 to DST11 and SRC20 to DST21.
The sink is a mapped input LOCAL, not merely a reached Tile.

Forward expansion reads SRC deadends.
Backward expansion reads DST deadends.
A deadend in the other direction must not reject a connection.
A direct destination landing does not require another jump.
That exception must preserve the existing deadend bit.

Only TraceRouteForward may add SRC deadends.
Only TraceRouteBackward may add DST deadends.
Learning requires all directional exits in the reached Tile to be blocked.
One remaining free directional exit prevents a new mark.
Lease restoration must retain learned deadend bits.

The first run succeeds despite preset SRC marks along the whole corridor.
The second run rejects a DST-marked jump whose landing is still transit.
The third run accepts the same DST mark when landing at the chosen source.
Transit controls must still reject marked directional jumps.

History: added for the directional-deadend and preemption separation change.
History: direct Moving completion is checked away from the old angle hint.
