# Fanout repair through an occupied final jump

Run `ctest --test-dir build -R '^fpga.fanout_final_jump$' --output-on-failure`.

The numeric fixture reproduces the recorded binding-24 failure in Example 1:
22 retained fragments, source `(12,25)/LOCAL154`, sink `(15,29)/LOCAL96`, and
two holes:

1. `(15,21)/DST263 -> (11,25)/DST98`.
2. `(15,32)/DST246 -> (15,29)/LOCAL96`.

It imports the actual seven-hop source-search prefix through Tiles `(16,30)`,
`(21,29)`, `(16,28)`, `(12,32)`, `(4,32)`, `(15,32)` to `(15,30)/DST193`.
The remaining physical path is:

```
(15,30)/DST193 -> SRC247 -> (15,29)/DST241 -> LOCAL96
```

SRC247 and DST241 belong to a foreign transit binding. The fixture preserves
its captured local fragments, including DST3537 -> SRC247 and DST241 -> SRC3808.
Unrelated portions leading to its remote endpoints are shortened. The large
device's unrelated topology and other source branches are not imported. All
node, signal, cell and crossbar names are synthetic; no busy bits exist without
registered routes. The failed route is installed from its captured partial
snapshot, rather than replaying the four earlier preemption transactions.

Before the fix, the regression reaches and expands DST193 at seven hops but
the production Fanout task fails: free search excludes SRC247, and terminal
preemption only examines already-reached SRCs or DSTs inside the sink Tile.
The new passing requirement is an independently checked eight-hop complete
route, a partial cut of the foreign transit, and exactly one requeued victim.

Additional cases check:

- Takeoff-owned SRC and Grounding-owned DST stay protected.
- A protected second terminal JOINT prevents all cuts, including the earlier
  eligible SRC and DST.
- The exception does not extend to arbitrary intermediate transit jumps.
- Disabled preemption, protected nets, cancellation, insufficient total hops,
  missing terminal connectivity and rejected cut previews leave routing intact.
- A wholly free alternative suppresses preemption.
- Two dependent fanouts sharing the contested jump are both partially cut and
  requeued; a sibling branching before the contested jump remains complete.
- A completed requester sibling retains the original nine-hop shared prefix
  when the failed binding is replaced with the eight-hop route.
- Ownership, leases and registration audits remain clean after each operation.

The implementation reuses the existing reached states and full-path cut
preview. It adds no search queue, persistent cache or increased state budget.
