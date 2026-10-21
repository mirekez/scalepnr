# Takeoff-preempted trunk incorrectly parked for Moving

Example 1 run 5 completed Basic and later failed after pass 27 with
`Fanout missing Generic trunk`. The affected source's trunk had completed
with 22 fragments. Another net's Takeoff then cut its transit, retaining a
gap from `(45,18)/DST25` to `(44,21)/DST66` and leaving 21 fragments.
The untouched sibling had no route yet. Its destination had 26 free terminal
alternatives; the handoff failed before attempting it.

The ordinary Takeoff callback passed `force_fanout=false` to
`unrouteVictimBinding()`. Unlike the Grounding callback, that function did not
inspect the retained route. Its Generic-labelled repair was parked in Moving
by pass finalization. The sibling waited in Fanouts, whose handoff only looked
for deferred Fanout gap repairs and therefore found none. Source history shows
no repair attempt between the cut and the final failure.

`fpga.fanout_takeoff_handoff` extends the real production full-flow fixture in
`fanout_trunk_handoff.cpp`. Two completed trunks precede the requester's remote
suffix and the victim's untouched sibling. The remote suffix forces ordinary
Takeoff preemption of a transit SRC outside immediate Grounding. All node and
cell names are abstract; all occupied resources have registered owners.
The original Grounding regression does not exercise this callback.

Before the fix, this case fails with the same handoff error. The companion
`fpga.fanout_takeoff_handoff_control` repairs the preserved victim and then its
sibling explicitly through the production batch API; it passes and proves a
legal bypass exists. The bypass rejoins beyond the landing newly owned by the
requester, rather than feeding a second driver into it.

The fix classifies a retained gap/shared-prefix repair from its actual route,
just as Grounding does. It does not relax the complete-trunk barrier, scan the
device, enlarge a search, or discard retained routing. After the fix both tests
must complete through normal orchestration, with requester/victim/sibling
routes complete, fixed placements unchanged, and every Tile ownership audit
clean.
