# Preempted trunk at the Basic / Fanouts handoff

`fanout_trunk_handoff.cpp` reduces the Example 1 run-4 failure to five
fixed cells on a 5-by-3 synthetic routing grid. Resource names are abstract.

Basic first completes a trunk with a deferred second sink. Another net's
Grounding then preempts a transit JOINT on that trunk and completes. The cut
preserves a gap and enqueues its repair as Fanout work. Basic now has no tasks,
so Moving sources also reaches zero. The broken handoff rejects the untouched
sibling for lacking a completed Generic trunk, before attempting the repair.
The original device trace cut a SRC, whereas this fixture cuts a JOINT; the
retained-gap notification and handoff failure are the same.

The fixture provides a free two-hop bypass. Before the fix:

- `fpga.fanout_trunk_handoff` fails with `Fanout missing Generic trunk`.
- `fpga.fanout_trunk_handoff_control` catches that specific failure and submits
  the retained repair followed by the sibling to the production routing batch.
  Both complete, proving that the barrier—not unavailable capacity—caused the
  failure.

After the fix, both cases complete through `routeDesign` without manual
intervention. Assertions require actual preemption, use of both bypass exits,
complete routes for both siblings and the preempting net, unchanged fixed
placements, and clean lease/owner/registration audits on every Tile.

The handoff now promotes one existing retained-gap repair per missing source
before releasing that source's siblings. It neither discards the trunk nor
treats a disconnected downstream suffix as a completed source tree. A missing
trunk with no queued repair still fails after Moving sources; an unsuccessful
repair still follows normal fail-fast routing.

Run with:

```sh
cmake --build build --target grounding_preemption -j2
ctest --test-dir build -R '^fpga.fanout_trunk_handoff' --output-on-failure
```

The separate `fanout_takeoff_handoff` cases exercise the ordinary Takeoff
callback during Fanouts. That callback formerly mislabeled a retained gap as
Generic work and parked it for Moving. See `fanout_takeoff_handoff.md`; the
Grounding-only fixture above cannot cover that repair-classification path.
