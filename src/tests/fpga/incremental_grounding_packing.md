# Incremental packing over an existing Grounding route

`incremental_grounding_packing.cpp` reproduces the terminal obstruction from
example 1's third run using only abstract routing-node names. The source CB,
destination CB and attached resource Tile are reduced to coordinates (0,0),
(1,0) and (2,0). This is a terminal/packing replay, not the whole design or its
long forward search.
An otherwise unused fourth Tile supports the transit and Takeoff controls.

The earlier run packs `old_lut` and installs a complete, registered route:

```
arrival_23 -> junction_23 -> junction_9 -> input_115
```

An independent direct path from `arrival_90` to `input_115` means the LUT has
**no mandatory JOINT reservation**, even though its live route uses
`junction_23`. The LUT is fixed/completed and its net is preserved.

The next run tries to pack `new_register` in the attached resource Tile.
Its sixteen incoming DST alternatives all require:

```
arrival_* -> junction_23 -> input_1
```

Ten of these DSTs are occupied and six are free, exactly as in the failure.
The surrounding occupancy also has complete routes and registered owners;
there are no anonymous busy bits. Each incoming DST has a resolved mapping.
`input_1` is free, but none of its sixteen paths can be leased because the
mandatory JOINT belongs to the earlier LUT Grounding, not transit.

## Checks

- `fpga.incremental_grounding_snapshot` verifies topology, sixteen entries,
  six free DSTs, zero free register paths, optional versus mandatory JOINT use,
  and non-transit ownership. The production congestion audit checks every
  Tile for orphan/missing leases and missing/stale route registrations.
- A positive control installs the older LUT route through its alternative
  arrival. The same resource Tile, cells and surrounding occupancy must then
  allow preview and exact register packing. Its JOINT is no longer occupied.
- `fpga.incremental_grounding_packing` additionally requires rejection from
  relaxed preview, strict preview and relaxed exact packing. These are separate
  fresh fixtures, so an accepted commit cannot contaminate the next check.
  Each check also verifies that the completed fixed LUT and its Grounding
  remain intact and repeats the ownership audit.
- Additional controls allow same-signal JOINT sharing and preserved transit,
  but reject foreign Takeoff and explicitly protected transit. They build
  complete registered routes and audit their node ownership as well.

Run with:

```
cmake --build build --target packing_test -j2
ctest --test-dir build -R '^fpga.incremental_grounding_' --output-on-failure
```

Before the fix, all three packing operations incorrectly returned position 0;
the regression failed while the snapshot and free-path control passed. With
the fix, both tests pass. The regression is not marked `WILL_FAIL`, disabled,
or changed to accept the buggy placement. The snapshot/control test remains
independent, so malformed setup cannot masquerade as reproduction of the bug.

The fix checks live owners only for occupied mandatory input JOINTs, including
in relaxed placement and previews. An earlier route's optional use is therefore
visible even when it has no mandatory reservation. Same-signal reuse and
eligible transit remain allowed; foreign terminal use and protected routes do
not. It reads live occupancy and adds no new cache.
