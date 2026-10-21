# Retained shared-input owner must be repaired before a new sibling

`fanout_shared_input_repair.cpp` reproduces the shared-input dependency found
in Example 1's fourth incremental run. It uses a 5×3 abstract fabric, seven
fixed cells and two signals. All routing-node names are synthetic `node_*`
identifiers. The source comments describe the topology and task order.

## Sequence

1. Run 1 routes `source_101` to `owner_102` and `stable_103`.
2. Run 2 creates another source's trunk in Basic. Its Fanouts task to
   `request_sink_203` preempts the first signal's transit at `(1,1)/JOINT33`.
3. The incomplete owner keeps its downstream `(3,1)/LOCAL80` input lease.
   Its repair is queued. `stable_103` stays complete, so the source still
   passes the completed-trunk check.
4. The next task, `sibling_104`, needs that same input. It occupies slot 1;
   the owner occupies slot 0. They share an input, not a physical cell slot.
5. The new sibling fails before the pending owner repair runs. All Tile
   ownership audits are clean: this is not fabricated congestion.

This is a reduced scheduler reproduction, not a full-chip topology copy.
The captured failure cut a transit SRC; this fixture cuts a transit JOINT
through the same production preemption/retained-gap machinery. It retains
one incomplete owner and one complete sibling, instead of the larger tree.
No test code manually leases nodes, creates gaps or implements route search.

## Checks and control

The normal test requires both full routing runs to succeed. Before the fix
it failed **after verifying the specific failure state**, rather than
accepting an arbitrary routing exception. It checks the completed preempting
fanout, surviving complete branch, retained gap, exact input owner, pending
repair and clean lease/registration audits.

The control starts identically and observes the same failure, then explicitly
submits the pending owner repair before the new sibling using `routeTaskBatch`.
The repair uses the free `(1,1) → (1,2) → (2,1)` bypass. Shared-input attachment
then succeeds. All five connections finish, endpoint placements are unchanged,
and audits remain clean. This proves that ordering alone can solve this fixture;
it does not prove that every corresponding full-chip repair is feasible.

Run with:

```sh
cmake --build build --target grounding_preemption -j2
ctest --test-dir build -R '^fpga.fanout_shared_input_repair' --output-on-failure
```

Reproduction result before a production fix:

- `fpga.fanout_shared_input_repair`: FAIL, pending shared-input owner repair.
- `fpga.fanout_shared_input_repair_control`: PASS, repair then reuse succeeds.

Each case has its own `build/tests/shared_input_repair*` working directory and
a 30-second timeout. Failure reports/images are written there. The failing
regression is not inverted with `WILL_FAIL`; it should become green when the
production router handles the dependency itself.

## Fix

Before building its completed-branch index, Fanouts checks leased candidate
inputs for retained, gapped owners from the same physical source. It repairs
one such owner using the existing gap-repair routine, then builds the index
and performs ordinary shared-terminal attachment. Foreign owners remain
protected. A failed prerequisite repair is reported against the owner and its
gap, rather than launching a futile new-sibling search. The pending repair
entry can subsequently be consumed as already complete.

The full-flow regression and control pass with the fix. A third case,
`fpga.fanout_shared_input_repair_blocked`, removes the bypass and verifies
explicit failure of the prerequisite repair while preserving the input owner,
the completed preempting route and the unaffected sibling. All three run with
the command above.

## Private terminal JOINT: a separate repair coverage gap

`fpga.fanout_private_grounding_repair` reproduces the next observed failure
mechanism. In the captured run, the retained terminal was
`(36,29)/DST1026 -> JOINT56 -> LOCAL64`. A read-only source search found a
seven-jump alternative reaching free `DST3841`, which also feeds `JOINT56`.
That JOINT and LOCAL belonged only to the disconnected owner. The repair
code permits replacing a private LOCAL's input, but rejects the equivalent
change at a private JOINT. The full-chip input-preserving audit hit its search
limit; that result alone does not establish that no other route exists.

The new case reduces this to the same 5×3 fixture, with two approaches present
before the initial routing:

- Old, shorter approach: source → contested JOINT33 → `(2,1)/DST20` →
  `(3,1)/DST220 → JOINT56 → LOCAL80`.
- Free alternative: source → `(0,2)` → `(1,2)` → `(2,2)` →
  `(3,1)/DST221 → JOINT56 → LOCAL80`.

The first run must use the old approach. The second run genuinely preempts
JOINT33 and requests the shared-input sibling, forcing production owner
repair. No test code fabricates leases or gaps, changes topology between
runs, or implements an alternative routing algorithm.

On failure the test checks the precise exception, retained gap, completed
preempting routes and unaffected sibling, private terminal ownership,
**all JOINT users including non-owning replicas**, and clean ownership audits.
Read-only reachability then proves that preserving the old input cannot
reach the sink (13 states, exhaustive), whereas replacing it can (15 states,
exhaustive). The printed path has no foreign owners and only the victim's
JOINT and LOCAL are occupied. This small reproduction is not a copy of the
full chip's search-budget or earlier preemption history.

`fpga.fanout_private_grounding_repair_direct` is the control: only the
terminal JOINT is removed, so both DSTs connect directly to LOCAL80. It must
finish the whole flow through the new approach. Both tests require all five
connections complete, the obsolete approach released, unchanged fixed
placements and clean final audits when routing succeeds.

Reproduction before the private-JOINT fix:

- Private-JOINT regression: **FAIL**, refuses the alternative terminal input.
- Direct-LOCAL control: **PASS**, repairs the victim and attaches the sibling.

The failure is deliberately not marked `WILL_FAIL`: the regression specifies
successful repair.
The existing scheduler regression stays green because its bypass rejoins the
old downstream DST and never needs to change a retained terminal JOINT input.
The separate docking-owner regression checks successful preemption and a
retained victim gap, not subsequent victim repair.

```sh
ctest --test-dir build -R '^fpga.fanout_(private_grounding_repair|shared_input_repair|docking_owner)' --output-on-failure
```

The new cases have separate `build/tests/private_grounding_repair*` output
directories and individual 30-second timeouts.

### Private-JOINT fix

Whole-source repair now permits changing the incoming arc of a privately held
JOINT in the sink Tile, using the same ownership and all-users check as the
private sink LOCAL. It does not relax intermediate-node, foreign-owner,
shared-replica or packed-input restrictions. Validation still precedes every
mutation; replacing the route releases the obsolete approach while keeping
the reused JOINTs and pin leased. The reachability diagnostic uses the same
terminal-input eligibility rule.

Both new cases now pass. `fpga.fanout_private_grounding` additionally exercises
a retained **two-JOINT** terminal with a changed incoming DST, including
same-net replicas, cross-net same-source aliases, foreign ownership, missing
arcs, cancellation, hop limits and packed intermediate inputs. Protected
cases must fail without changing routing state; the private case must repair
and retain exactly the required terminal resources.
