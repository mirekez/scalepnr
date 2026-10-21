# Source ordering: LOCAL/DST cache collision

This regression isolates the candidate-enumeration failure from example 1,
run 4 Basic routing. It uses only abstract names: `drive_161`, `arrival_161`,
`exit_*` and `landing_*`.

The fixture preserves the source `(54,68)`, destination `(54,59)`, LOCAL ID
161, all 24 numeric source alternatives, all 16 occupied SRC bits, and the
empty deadend mask from the recorded failure. Among the 24 exits, 23 are free
and only exit 248 is occupied. All exits have resolved synthetic jumps.
This is a Crossbar-ordering unit test, not a full routing/ownership fixture.

The controlled earlier query is for DST161 in the same direction. Its two
exits are disjoint from LOCAL161's exits. The real failure did not log which
earlier query populated the cache, so this donor list is deliberately synthetic,
not claimed to be an exact reconstruction of that earlier route.

The old production cache key overlapped node-role bits with direction bits:
`(target_key << 14) | (role << 12) | node`. For this direction, target_key is
105. LOCAL's role is 0 and DST's is 4; both requests get key 1720481.

Four independently runnable checks are registered:

- `fpga.source_order_snapshot`: cold LOCAL/DST queries must have the correct
  independent exit sets; the snapshot must have 24 eligible/23 free exits.
- `fpga.source_order_dst_first`: query DST, then LOCAL. LOCAL's exact ordering
  must match its independent cold control. The faulty implementation returns
  the two DST exits and leaves zero active LOCAL candidates, as in the real
  failure.
- `fpga.source_order_local_first`: reverse the calls. DST's order must remain
  independent too; the faulty implementation returns all 24 LOCAL exits.
- `fpga.source_order_directions`: 900 LOCAL/DST queries across 225 directions,
  repeated in reverse direction order, must match a global-order oracle that
  bypasses the per-node cache.

Both collision checks require correct behavior and fail on the buggy code.
They are not marked `WILL_FAIL`; the snapshot control remains a passing test.
The fix shifts the direction field by 15 instead of 14, leaving three separate
bits for node role and twelve for node ID. Compile-time checks guard the field
sizes. It changes no candidate ordering or routing policy and adds no runtime
search work.

```sh
cmake --build build --target source_order_collision_test -j2
ctest --test-dir build -R '^fpga.source_order_' --output-on-failure
```
