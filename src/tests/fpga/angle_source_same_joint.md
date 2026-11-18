# angle_source_same_joint

Purpose: retry another SRC through the same JOINT after one exit fails.
Implementation: angle_gap_priority_case(5) in fanout_gap_replay.cpp.
Run the fpga.angle_source_same_joint CTest case.

The retained prefix arrives at (10,2)/DST12.
JOINT31 connects the arrival to SRC30 and SRC34.
Angle priority selects SRC30 before SRC34.
SRC32 is a lower-priority alternative through JOINT33.
SRC30 reaches (16,2)/DST50 and continues through SRC70.
SRC70 reaches (17,2)/DST71 and LOCAL72.
LOCAL72 has no outgoing arc and is not the requested sink.
SRC34 instead reaches an intermediate Tile with a usable sink connection.
All topology and names are abstract.

The decision log must show SRC30 attempted before SRC34.
The shared JOINT must remain available after the first recursive failure.
The completed route must use SRC34 and three physical jumps.
The test does not change the angle ordering to obtain that result.
Source repair reads persistent SRC marks but must not create them.
Neither the failed SRC30 nor the successful SRC34 gains a mark here.
Immediate cancellation preserves leases and existing marks.
A one-state cap must leave the untried exit unmarked.
Route and ownership audits check the final binding.
Teardown verifies that no private resources remain occupied.

History: added to catch losing the parent JOINT cursor after child failure.
History: repair traversal moved from Docking to RoutePreemption.
History: only TraceRouteForward and TraceRouteBackward now learn marks.
