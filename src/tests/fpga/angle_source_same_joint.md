# angle_source_same_joint

Purpose: retry another SRC through the same JOINT after an exhausted exit.
The angle-guided source trace must preserve the remaining JOINT choices.
Implementation: `angle_gap_priority_case(5)` in `fanout_gap_replay.cpp`.
Run: `ctest --test-dir build -R '^fpga.angle_source_same_joint$' --output-on-failure`.

The retained prefix arrives at (10,2)/DST12 from the source Tile.
JOINT31 connects that arrival to SRC30 and SRC34.
Angle priority selects SRC30 before SRC34, then SRC32 through JOINT33.
SRC30 jumps to (16,2)/DST50 and continues through SRC70.
SRC70 reaches (17,2)/DST71, whose only continuation is LOCAL72.
LOCAL72 has no outgoing physical arc and is not the requested sink.
This proves that the SRC30 branch cannot complete the connection.
SRC34 instead reaches (11,3), which has an exit to the sink Tile.
SRC32 offers a lower-priority route through (3,2).

The trace log must show SRC30 attempted before SRC34.
The shared JOINT must remain usable after its first SRC fails.
The successful route must use SRC34 and contain three physical jumps.
SRC30 must immediately retain its proved deadend mark.
SRC34 must remain unmarked because its continuation reaches the sink.
Immediate cancellation must preserve leases and existing deadend bits.
A one-state search cap must not mark an untested exit as dead.
Route and ownership audits check the completed binding and its leases.

History: blanket handling of rejected edges suppressed genuine deadends.
The physical LOCAL leaf was incorrectly classified as unfinished search.
Eligibility now distinguishes proved unusable nodes from policy rejection.
History: Docking now calls Wire's shared ordering; expected choices are unchanged.
