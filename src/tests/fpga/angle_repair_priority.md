# Angle-priority route repair

`grounding_preemption angle_gap_priority` uses one small physical fixture with
live route bindings and retained source and terminal leases. Its node names are
synthetic; each tested jump resolves to a concrete tile and DST node. Each
mode checks cancellation leaves the route and leases unchanged, verifies the
completed physical path, audits ownership, and unroutes the result.

- Mode 0: a raw low-number SRC opens 18 westward destinations, while a
  higher-number SRC reaches the sink to the east through a JOINT. With a
  16-label gap budget, the legacy FIFO walk filled its frontier and invoked
  source fallback. Angle-first repair reaches the east path in two labels.
- Mode 1: the east path crosses an intermediate LOCAL and JOINT. The old
  CROSSBAR-only gap walk cannot represent that bridge. This checks the typed
  source completion remains available without repeated fallback calls.
- Mode 2: both exits point east. The shorter SRC is reached through a JOINT
  and one intermediate tile; the longer direct SRC reaches the sink tile in
  one jump. The source-only search must consider the shorter angle-priority
  exit first and commit its path. A trace checks candidate order independently
  of the completed-route assertion.
- Mode 3: the best-angle SRC is occupied by a complete foreign route. A worse
  free SRC shares its JOINT, while a medium-angle free SRC uses another JOINT.
  The medium exit must be considered and committed before the worse exit;
  the foreign owner must remain unchanged through requester teardown.
- Mode 4: a preferred branch reaches a shared DST at its hop limit. A second,
  shorter branch reaches the same DST with one hop left and can complete.
  The failed first visit must not mark that DST globally exhausted.
- Mode 5: a free preferred SRC dead-ends after a shared JOINT. Its fully
  exhausted exit is marked in the tile's persistent SRC mask, then a second
  SRC through the same JOINT completes. Immediate cancellation changes no
  mask; a capped search does not mark an untested third exit.
- Mode 6: a free sink LOCAL is reached immediately despite many unrelated
  outgoing SRCs. Cancellation changes no leases or deadend masks.
- Mode 7: the preferred free SRC carries a preset deadend bit. Source tracing
  skips it, commits the alternate physical route, and preserves the mark.

Direct selectors:

```text
build-release/grounding_preemption angle_gap_priority 0
build-release/grounding_preemption angle_gap_priority 1
build-release/grounding_preemption angle_gap_priority 2
build-release/grounding_preemption angle_gap_priority 3
build-release/grounding_preemption angle_gap_priority 4
build-release/grounding_preemption angle_gap_priority 5
build-release/grounding_preemption angle_gap_priority 6
build-release/grounding_preemption angle_gap_priority 7
```

CTest names are `fpga.angle_gap_joint_priority`,
`fpga.angle_gap_local_bridge`, and `fpga.angle_source_length_priority`.
The additional cases are `fpga.angle_source_blocked_priority`,
`fpga.angle_gap_revisit_priority`, `fpga.angle_source_same_joint`,
`fpga.angle_terminal_first`, and `fpga.angle_source_preset_deadend`.
