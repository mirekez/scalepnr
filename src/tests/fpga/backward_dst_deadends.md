# backward_dst_deadends

Purpose: make backward angle tracing symmetric through persistent DST marks.
Run: ctest --test-dir build -R "^fpga.backward_dst_deadends$" --output-on-failure.
Implementation: backward_dst_deadends_regression in fanout_gap_replay.cpp.

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
Learning requires all directional exits to have persistent deadend marks.
A busy lease can later unwind or be preempted, so it cannot prove a mark.
Lease restoration must retain learned deadend bits.

Moving checks placement eligibility independently of free transit.
An ineligible intermediate Tile receives no placement callback.
SRC-mark controls prove forward marks cannot block backward tracing.
Transit controls must still reject marked directional jumps.

History: added for the directional-deadend and preemption separation change.
History: added eligible endpoint and ineligible transit Moving controls.
