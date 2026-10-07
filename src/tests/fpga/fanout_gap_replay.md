# Two-state fanout repair reproduction

Run with:

```sh
ctest --test-dir build -R '^fpga.fanout_gap_replay$' --output-on-failure
SCALEPNR_DEBUG_ROUTE_REPAIR=1 build/grounding_preemption gap_replay
```

This is a deterministic reproduction of the recorded local failure, not a
full-device design import. Numeric coordinates, the victim's six original
paths, both cuts of the failing path, and all 24 outgoing choices at the two
reached states are retained. Node and signal names are synthetic. Unrelated
owner paths outside this neighborhood are shortened synthetic routes with
actual endpoint pins and registered ownership; no bare busy bits are used.

The failing binding starts complete with 18 fragments. The production cut
preview and atomic replacement functions reduce it to 16 and then 15
fragments while completing both requesters. It retains these gaps:

1. `(16,28)/DST242` to `(15,25)/DST241`.
2. `(12,14)/DST256` to `(12,17)/LOCAL106`.

The final recorded source tree has four complete bindings and two partial
bindings. The independent partial sibling, including its later two cuts, is
also reproduced. This corrects the earlier five-complete/one-partial count,
which came from an intermediate rather than final history snapshot.

The first repair reaches exactly:

```text
(16,28)/DST242 --SRC1026--> (22,28)/DST1025
```

At the first state, seven SRCs are occupied and one is free. At the second,
13 SRCs are occupied; its three free SRCs reach occupied landings at
`(26,28)/DST1025`, `(16,32)/DST3377`, and `(21,28)/DST3847`. All three
landings are held by explicit partial owner routes. The first has four owning
bindings. All blocked resources belong to foreign sources.

The test checks:

- Production preemption creates the expected gaps, rather than injecting gaps
  directly into the victim.
- All occupancy has actual registered owners before and after preemption.
- An independent traversal finds exactly two free reachable states, 20 busy
  SRCs, one free landing, and three busy landings.
- Production repair fails at its default 32,768-state/200-hop budget and with
  doubled budgets; the production Fanout task entry also returns failure.
- The repair's diagnostic observer reports the same six inspected tiles.
- Failed repair does not change any retained route, lease or registration.
- The existing misleading sink fallback `(12,17), node=-1` is reproduced.

## Checking the proposed wider repair

An independent, read-only traversal starts at the driver `(25,28)/LOCAL149`,
not at the cut boundary. It allows reuse of every resource owned by the same
source, ignores joint occupancy, and accepts any DST arrival at the sink Tile
without requiring a Grounding connection. This intentionally overestimates
the available paths; foreign occupied SRCs and DSTs remain blocked.

Before preemption this check reaches both the future gap endpoint and sink
Tile. After preemption it exhausts **30 states** and reaches neither
`(15,25)/DST241` nor the sink Tile `(12,17)`. The traversal does not change routing.

Consequently, widening the search cannot solve the original reduced fixture
under unchanged foreign occupancy. This conclusion does not establish that
the complete real device lacks a bypass: topology outside the captured paths
and local frontier was not imported in that original fixture. The later
real-path import described below adds the verified missing topology.

The same test now exercises `auditNetRouteReachability()`, a node-level
diagnostic usable on the loaded device at failure time. Unlike the relaxed
test-only upper bound, its default mode preserves existing input arcs of
same-source occupied nodes and checks actual JOINT occupancy. It traverses
LOCAL/DST/JOINT/SRC connectivity and physical jumps from the driver's LOCAL,
reporting paths to every retained gap end and the sink LOCAL. The log records
state-limit exhaustion separately from unreachable goals and prints every
path node's busy flag and owners. It never changes routes or leases.

To run it on a real failed repair, set
`SCALEPNR_AUDIT_SOURCE_REACHABILITY` to a substring of the physical net name
and `SCALEPNR_SOURCE_REACHABILITY_LOG` to an output filename. If the
input-preserving search cannot reach the sink, the log also includes a relaxed
same-source-input search; paths from that second mode are not certified to
preserve existing branches.

## Verified real-device bypasses

The full replay in `tests/prjxray/1/build/source_reach.crBFW3Hj` used the same
netlist, constraints, 2,000-cell run size and 600-second routing-stage timeout.
Its failure report is byte-identical to the earlier report (SHA256
`6b262853bae4174bad13ddfc781e19f649e2a3bffcbc72724a78e95a5e787153`).
The selected physical net ends in `162539`, binding 2. At the failed repair,
the input-preserving full-source audit discovered 682,667 node states, did
not hit its limit, and found both goals. Every new resource on the printed
paths is free; occupied path nodes belong to the same source. No foreign
lease or retained input arc was changed.

`Fixture::real_bypasses()` imports the exact numeric connections from that
audit after the original reduced-fixture assertions. It adds no busy bits,
cells, physical resource releases, or guessed jumps. All names remain
synthetic. Two paths are available:

1. **First gap end, nine physical hops from the source.** Reuse the existing
   tree through `(22,29)/DST3843`, `(21,30)/DST3619` and `(25,34)/DST819`.
   Then follow:

   ```text
   (25,34)/DST819 -> LOCAL165 -> SRC3078
     -> (15,34)/DST3079 -> SRC3078
     -> (4,34)/DST3079 -> SRC3617
     -> (4,35)/DST545 -> SRC977
     -> (11,31)/DST977 -> SRC977
     -> (15,27)/DST977 -> SRC247
     -> (15,25)/DST241  [retained gap end]
   ```

2. **The actual sink, ten physical hops.** This separate alternative uses a
   free source exit rather than an existing branch:

   ```text
   (25,28)/LOCAL149 -> SRC1025
     -> (36,28)/DST1028 -> SRC192
     -> (36,25)/DST192 -> SRC3072
     -> (25,25)/DST3077 -> SRC3072
     -> (15,25)/DST3077 -> LOCAL178 -> SRC3376
     -> (11,30)/DST3376 -> SRC3074
     -> (4,30)/DST3072 -> SRC161
     -> (4,23)/DST161 -> SRC977
     -> (11,19)/DST977 -> SRC737
     -> (12,18)/DST737 -> SRC242
     -> (12,17)/DST246 -> LOCAL106  [reserved sink]
   ```

The intermediate LOCAL165 and LOCAL178 connections were separately checked
against the loaded connection specifications. They are routing wires with
incoming and outgoing routing arcs, not added logic-cell passthroughs. The
sink uses a direct DST246-to-LOCAL106 connection, with no terminal JOINT.

After import, the exact cut boundary still has only two reachable states, but
**the production Fanout entry must now complete the binding**, using the
verified ten-hop source-to-sink bypass. No artificial shortcut is added.
An independent checker verifies every numeric arc, path continuity and hop
count, rather than trusting only the route-complete flag.

Removing the incoming arc to LOCAL165 makes only the first diagnostic goal
unreachable; removing the incoming arc to LOCAL178 makes the sink unreachable
and production repair must fail without changing routing. Cancellation, a
one-state limit and a nine-hop limit must also fail without mutation. Reserving
the intermediate LOCAL as a packed input also prevents repair.

The successful repair leaves foreign routes and all other bindings unchanged,
including the independent partial sibling. LOCAL178 is leased as a routing
wire, not a cell pin. Repeated transit preemptions exercise its SRC, LOCAL,
incoming DST and landing DST independently. Each cut preserves the remaining
route and releases its intermediate LOCAL; production repair must then succeed
again. Takeoff and Grounding remain protected. Finally, removing the repaired
binding must release its resources cleanly. Ownership/registration audits run
throughout.

The fallback is bounded by the ordinary repair state/hop limits and is invoked
only after exact-gap repair fails. It searches from the actual driver using
numeric connectivity, preserving inputs of occupied same-source nodes. It
validates the entire path before registering the replacement and releasing
the old binding's unneeded claims. This regression proves the captured bypass
works; it does not assert that the full Example 1 design finishes routing.

## Second failure: reached source path blocked at terminal transit

`fpga.fanout_gap_grounding` replays the next failure using synthetic names.
It starts from the recorded 23-fragment route from `(22,32)/LOCAL156` to
`(11,16)/LOCAL85`, applies real transit cuts to retain three holes, and installs
owned blockers on all eight exits of the first hole at `(15,29)/DST244`.
The retained numeric source bypass passes through `(12,33)/DST23 -> LOCAL165
-> LOCAL184 -> SRC162` and eventually reaches `(11,14)/SRC66`. Its final jump
lands on `(11,16)/DST66`, which directly feeds the reserved LOCAL85 but belongs
to foreign transit. Free-only repair exhausts the fixture without completion.

The foreign transit continues from DST66 through SRC98 to `(11,22)/DST98`.
The fixture extends this into two dependent bindings to additionally test
shared-cut atomicity and requeueing. Remote owner prefixes are synthetic; this
is a replay of the critical connectivity and ownership, not the entire chip.

The production `RouteDesign::routeNetTask` must now complete the requester,
retain the victim islands, and queue both affected bindings. Disabling
preemption, protecting the owner, or removing the terminal connection must
leave all routing state unchanged. A second variant adds a free terminal
alternative, which must suppress preemption. A third uses two JOINTs and
checks that a protected second JOINT or packed JOINT reservation prevents
the otherwise eligible DST cut. Every case audits leases and registrations.
