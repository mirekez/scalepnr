# Reached retained suffix rejected at the search-state limit

Run `ctest --test-dir build -R '^fpga.fanout_retained_suffix$' --output-on-failure`.
The implementation is `fanout_retained_suffix_regression()` in
`fanout_gap_replay.cpp`, sharing the existing numeric routing fixture.

This regression requires successful fanout repair at the unchanged default
budget of 32,768 states. It was red before retained-suffix reconnection was
implemented. The repaired search completes this fixture with 33 generated states.

## Captured behavior

The fixture starts with the recorded 17-fragment complete route. Actual partial
preemption creates the gap `(15,29)/DST244 → (16,15)/DST978`. Six busy exits and
two busy landings trap exact-gap repair. Every busy resource has a registered
foreign route owner; congestion audits check leases and registrations.

Whole-source search follows the recorded nine-hop bypass, including three
intermediate LOCAL bridges, and reaches `(16,15)/DST978`. Four intact physical
hops remain in the binding:

```
(16,15)/DST978 → SRC248 → (16,14)/DST242
              → SRC3809 → (15,13)/DST3809
              → SRC3616 → (12,14)/DST3616
              → SRC16 → (12,15)/DST19 → LOCAL80
```

Before the fix, production search followed the first two retained hops, then refused
`(15,13)/SRC3616` solely because its state array is full. The test checks the
production trace for the reached nodes, costs, hop counts and state limit.
The fixed search validates and appends the four-hop tail as soon as it reaches
`(16,15)/DST978`, before expanding unrelated fabric. The success trace must show
this exact reconnection, nine prefix hops and four suffix hops.
The test exercises `RouteDesign::routeNetTask`, not a replacement search implementation.

## What is exact and what is synthetic

Coordinates, numeric arcs of the original route, gap boundaries, trapped
frontier choices, source bypass and retained suffix match the captured failure.
All textual cell, signal, Tile and node names are synthetic (`node_<hash>`, etc.).
Foreign owners have shortened synthetic paths outside the relevant frontier.
A completed sibling shares the first two physical hops to test shared ownership.

Unrelated full-device search load is **not** copied. Eight synthetic dead-end
Tiles each expose 4,096 free LOCALs. Their search priority makes the unchanged
default state limit fill after the second retained-hop arrival is queued and
before it is expanded. This reproduces the same stopping edge, not the original
full-chip queue ordering or exact state indices.

## Controls

- An independent, input-preserving reachability audit finds the complete path.
- Removing the final suffix-to-sink arc makes the sink unreachable: no accidental
  alternate completion exists in the fixture, and production repair must fail.
- A 12-hop limit rejects the complete 13-hop route, including its retained tail.
- Cancellation leaves routing unchanged.
- Small retained-tail fixtures reject a third visit to a Tile but accept two;
  they also reject a missing second-JOINT arc and a packed-input JOINT reservation.
- A cyclic tail returning to the source is rejected with only the source state
  available. With a larger search budget, a separate one-hop, loop-free path
  succeeds; the invalid tail does not suppress ordinary search.
- Failed repair leaves all route fragments, leases and registrations unchanged.
- With 65,536 states, the same topology completes a verified 13-hop path.
  This is only a reachability control, not a production fix or limit change.
- Successful repair leaves the sibling's physical path and all foreign owners
  unchanged. Shared-prefix primary ownership may transfer normally; congestion
  auditing checks that the transferred leases remain correct.

The final assertion still requires the original default-budget call to succeed;
the larger-budget control cannot turn the regression green.
