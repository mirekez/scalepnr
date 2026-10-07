# A private retained Grounding blocks its own replacement

Run:

```
ctest --test-dir build -R '^fpga.fanout_private_grounding$' --output-on-failure
```

Implemented in `fanout_private_grounding_regression()` in
`fanout_gap_replay.cpp`. This regression failed before private-sink replacement
was implemented. It now requires successful replacement at the unchanged
production search budget, plus rejection when another binding needs the sink.

## Captured situation

The failing binding is represented by 27 retained fragments and two gaps:

```
(15,29)/DST244 -> (11,22)/DST3538
(11,21)/DST242 -> (16,21)/DST1024
```

Its final Grounding is still leased:

```
(12,37)/DST3848 -> JOINT25 -> JOINT11 -> LOCAL111
```

Only the failed binding owns this sink and its old Grounding. A completed
sibling shares the source prefix but does not use the sink. The recorded free
alternative has six physical hops:

```
(22,32)/LOCAL156 -> SRC19
 -> (22,33)/DST22 -> SRC3079
 -> (16,33)/DST3075 -> SRC3079
 -> (12,33)/DST3075 -> SRC3379
 -> (5,37)/DST3379 -> LOCAL128 -> LOCAL184 -> SRC3078
 -> (11,37)/DST1030 -> SRC260
 -> (12,37)/DST263 -> LOCAL111
```

Before the fix, `RouteDesign::routeNetTask` reached and expanded DST263 but rejected the last
edge because the private sink's retained input is JOINT11 rather than DST263.
The fixed private-sink case must complete this path and release the old private
Grounding. Protected cases still check the production trace for that reached
six-hop arrival, not only independent graph reachability.

## Fidelity and deliberate reduction

Coordinates, numeric arcs, both gap boundaries, the complete retained path,
two-JOINT Grounding and alternative route come from the recorded failure.
Names are synthetic: `node_<hash>`, `signal_<number>`, `cell_<number>`, etc.
No device-vendor node names or external databases are needed.

The full-chip search background, unrelated blocked exits and other branches
are omitted. Retained fragments have consistent leases, and one completed
synthetic sibling represents source-prefix sharing. The failed binding is
renumbered to zero. Before the fix the small fixture exhausted its graph rather
than its state budget: it isolates the same incorrect terminal-input check,
not the full-chip queue count or preemption history.

## Controls and assertions

- All occupancy is backed by actual registered routes and checked by congestion
  audits. The sink is owned only by the binding being replaced.
- Strict reachability and production agree: the private sink can change input;
  a preserved sibling, alias, or foreign owner prevents this replacement.
- Shared sibling and alias cases deliberately have only one live sink owner.
  Shared non-owning references must also protect their incoming connection.
- Removing DST263's direct arc makes even relaxed reachability fail.
- Failed repair changes no route fragments, leases or registrations.
- Doubling the search budget cannot bypass the preserved-input restrictions.
- Cancellation, a five-hop limit, and a packed intermediate LOCAL reservation
  all reject without modifying routing.
- A separate fixture omitting the old retained tail, but keeping the same sink
  pin leased, successfully routes the same six-hop alternative. The failing
  fixture is never modified to help it pass.
- Successful replacement must preserve the sink pin, keep the completed
  sibling routed, perform no preemption, and release old private DST3848 and
  JOINT25/JOINT11. An independent path checker verifies all six physical hops.

The final assertion requires success in the fixture **with** the old Grounding.
Passing the positive control cannot hide the reproduced failure.
