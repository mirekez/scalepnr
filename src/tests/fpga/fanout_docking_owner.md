# Fanout Docking: same-Tile transit misclassified as an endpoint

## Occupied forward-frontier coverage

`fpga.fanout_docking_forward` extends this fixture with a 13-hop approach
inside the destination window. Each search direction retains its production
10-hop limit. The first useful forward exit and a middle hop are occupied
by separate, genuinely committed transit owners. A free but useless departure
keeps branch collection from hiding the Docking behavior. All names remain
synthetic, and no search budget is increased.

Before the fix, the occupied-exit case fails while the free control passes:
forward provisional search does not invoke `eligible_edge`, so the reverse
half alone cannot cover the connection. The fix applies the same speculative
eligibility check in both directions. Actual cuts still occur only after
validation of the entire joined replacement.

Additional controls cover a protected Grounding owner and missing topology
after an eligible obstruction. Both must fail without changing any physical
leases. Every case audits all Tiles, including successful preemption victims.
This reproduces the forward-search asymmetry observed during the newer
Example 1 failure; it does not claim to prove that this synthetic path exists
in the complete real-chip snapshot.

`fpga.fanout_retained_fork_landing` separately constructs the observed
`pos=FORK, owns_dst=false, owns_landing=true` state through a real partial cut.
It checks landing ownership and leases after cutting, sibling removal,
repair, and final teardown. It does not manually remove a lease to manufacture
the captured inconsistency.

The first retained-fork lifecycle run is green without a lease-code change;
it guards the observed ownership shape but does not yet reproduce the real
missing-bit event. That separate event must be traced to its mutating
operation before claiming a bookkeeping fix.

The subsequent replay traced the missing bit to `repairFromSource`.
`fpga.fanout_fork_source_replacement` reproduces that operation with nested
fanouts: the departing branch owns the branching DST in its first private
TRANSIT fragment, while the surviving sibling's FORK borrows it. Cutting
farther downstream forces a new route from the driver. Before the fix the
replacement succeeds but clears the surviving sibling's branching DST.
The test fails on that exact missing lease, not on an invented busy mask.

Prefix promotion now transfers that next-fragment incoming DST to the
surviving prefix's landing before old-route cleanup. This is a constant-time
boundary check in the existing transfer loop. The regression checks complete
routes, all Tile ownership audits, and release of the transferred landing
when the surviving branch is eventually removed.

The combined Example 1 replay also verifies the fix: the watched branching
DST remains leased, and subsequent owner lookups return the surviving
binding with `owns_landing=true`. The original forward-Docking failure is
passed; run 4 completes passes 10 and 11 before a later, separate shared-input
gap-repair failure. This is not a claim that the full example now routes.

Final validation of these changes passed all 115 non-puzzle regressions and
all six routing puzzles with unchanged limits. The three large routing cases
finished in 247.73s, 249.64s, and 250.45s. The five placement puzzles were not
rerun in this iteration.

`fpga.fanout_docking_forward_preserve` gives the early branch an occupied
13-hop completion and a later source-local alternative a free completion.
Both transit owners must remain complete: the whole task must exhaust its
established choices before enabling the new forward-preemption fallback.
Eager expansion is not acceptable merely because its selected route works;
it can unnecessarily displace routes that a later existing choice preserves.

The existing late-predecessor test in `docking_test.cpp` covers both local
and outside-window reverse completion. Both preserve the exact 32-candidate
backward quota check, including 31/32/33/40-decoy cases. The local case also
checks that new forward possibilities do not displace an established join.
Eligibility accounting separates local/expanded terminal checks from the
bounded transit checks.

`fanout_docking_owner.cpp` uses abstract node names and a synthetic 10×10
fabric. It runs the production Fanouts/Docking implementation, not a second
router or a test copy of the preemption predicates.

## Captured situation

An incremental-run fanout has completed source-tree branches. Its new sink is
free, but the approach crosses several foreign transit routes, including a
retained incomplete route. Free Docking fails; provisional Docking reaches a
complete candidate and final validation rejects it with `endpoint_owner`.

The focused replay identifies the contested resource as `(57,36)/DST26`:
physical owner present, transit classification false, net preemptible. The
fixture translates the approach by (-50,-33), to `(7,3)/DST26`, and uses
synthetic owner names. It is a reduced mechanism test, not a database copy or
a claim that all routing congestion on the original chip has been reproduced.

A second focused replay confirmed the **actual rejected fragment**:
`from=(57,36), to=(57,36), local=26, src=23, dst=23, pos=1,
shared=false, owns_dst=true, owns_landing=true`. Thus the same-Tile
continuation is observed evidence, not merely a synthetic hypothesis.

## Fixture and assertions

A completed trunk provides the starting DST `(7,7)/3079`. A free departure
ends without reaching the requested sink. The useful six-jump approach is:

```
(7,7) -> (7,1) -> (7,2) -> (7,3) -> (7,4) -> (3,4) -> (3,6)/LOCAL95
```

Five independent owners occupy parts of that approach. The contested owner
enters `(7,3)/DST26`, traverses `SRC23 -> DST23` **inside that same Tile**, and
then leaves toward its own sink. Neither of its endpoint cells is on `(7,3)`.
This is a fabric continuation: it has a SRC and landing DST, and no LOCAL.
The owner retains two gaps after earlier cuts. Its seven-fragment structure
matches the captured owner:

1. Source pin at `(0,6)`.
2. Takeoff to `(4,6)`, with its arrival retained.
3. Gap from `(4,6)` to `(7,2)`.
4. Incoming hop `(7,2)/DST195 -> SRC22 -> (7,3)/DST26`.
5. Same-Tile hop `(7,3)/DST26 -> SRC23 -> DST23`, with its arrival retained.
6. Gap from `(7,3)` to the sink at `(3,5)`.
7. Retained sink pin.

Initial routes are committed and registered through production functions.
Incomplete routes are produced by the production partial-unrouting routine;
the test does not fabricate busy masks or gaps. All Tiles are audited before
and after the request. Failed attempts must leave physical lease masks
unchanged, and fixed endpoint positions must not move.

The observed mismatch is that physical-owner lookup finds the owner, while
transit-only lookup excludes it because its fragment has `from == to`.
The main regression requires the requester to complete. It failed before the
fix and is not registered with `WILL_FAIL`.

The first failing gate is `routeUsesNodeOnTile(..., transit_only=true)` in
`Wire.cpp`, reached from `preemptDockingBridge`. Partial-cut planning also has
same-coordinate exclusions in `transitCutEnd`; merely bypassing the first
rejection is not sufficient evidence that a safe, useful cut can be committed.
The regression requires the entire production request to succeed and keeps
the genuine-Grounding control to guard against weakening endpoint protection.

## Controls

| CTest | Expected result |
|---|---|
| `fpga.fanout_docking_owner` | Request completes despite the same-Tile transit owner |
| `fpga.fanout_docking_owner_free` | Same useful approach without foreign owners succeeds |
| `fpga.fanout_docking_owner_transit` | Ordinary inter-Tile transit owners succeed |
| `fpga.fanout_docking_owner_removed` | Removing only the contested owner permits the request |
| `fpga.fanout_docking_owner_grounding` | Genuine Grounding remains protected; rejection changes no leases |

Run all five with:

```sh
cmake --build build --target grounding_preemption -j2
ctest --test-dir build -R '^fpga.fanout_docking_owner' --output-on-failure
```

For the exact rejected cut and owner fragments, run the main case with
`SCALEPNR_DEBUG_ROUTE_NET=request_owner`. The extra production logging is
opt-in and does not modify acceptance or preemption policy.

## Verified result before a fix

The main regression fails with `physical=1 transit=0` and production
`endpoint_owner` rejection; all four controls pass. All 101 existing
non-puzzle regressions also pass (105 passing cases including these controls).
The final Example 1 replay reaches the same run-4 failure and the same
`(62,25)/DST98` reported search stop, without timeout or stagnation. Its target
backward-state report matches the earlier capture after excluding the header.
The new regression's node audit is clean, so it isolates this classification
defect from the separate missing-lease observations in the wider chip audit.

## Fix

Owner classification and partial-cut planning now recognize a fabric hop by
its SRC and landing DST, including zero-distance hops. Takeoff and terminal
Grounding remain protected. The direct transit-victim selectors use the same
rule. No speculative cutting or extra search is introduced.

The regression checks incoming DST, SRC, and retained landing ownership,
previews the partial cut, and requires successful production routing with a
clean post-route congestion audit and retained victim endpoints.

All five cases pass after the fix. The full 117-test suite was exercised,
including all 11 puzzles. Two older fixtures omitted the landing DST on their
synthetic transit hops; these now describe complete numeric hops. The old
same-Tile negative owner-query control now describes an actual Takeoff, not a
fabric continuation. Both corrected regressions and the five new cases passed
together on the final recheck; the other 115 cases passed in the full run.
