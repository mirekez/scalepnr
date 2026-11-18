# directional_deadend_destination

Purpose: allow marked SRC jumps to finish locally in the destination Tile.
Run: ctest --test-dir build -R "^fpga.directional_deadend_destination$" --output-on-failure.
Implementation: directional_deadend_destination_regression in fanout_gap_replay.cpp.

The fixture uses only abstract numeric routing nodes.
It contains a source Tile, an intermediate Tile, and a sink Tile.
Physical jumps connect SRC10 to DST11 and SRC20 to DST21.
A fixed resource Tile may map its sink LOCAL to a different routing CB.

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
Marking controls distinguish one free SRC from an entirely blocked SRC mask.
A return-to-destination detour must fail after bypassing a marked jump.

History: added for the directional-deadend and preemption separation change.
History: added eligible endpoint and ineligible transit Moving controls.
