# Rejoining a retained tail after a transit gap

`fanout_shared_terminal_gap.cpp` exercises production gap repair, route
commits, partial cuts, route registration and congestion audits. Node labels
are generated as `node_<number>`; no device-specific vocabulary is required.

## Captured situation

Example 1 stopped at `Fanout shared-input owner repair` in run 4, without a
deadline or stagnation expiry. The input owner retained a gap from
`(49,41)/DST3078` to `(57,39)/DST1029`. Two interrupted bindings shared
`(53,40)/DST545 -> JOINT55 -> LOCAL63`. The terminal audit reported two claims,
one physical owner and no missing leases or stale registrations.

The initial reduced fixture translates these boundary and terminal coordinates by
`(-49,-38)` and reduces the intervening fabric. It commits an owner route,
attaches a shared replica, cuts both through the production partial-unrouting
API and commits an unrelated Grounding on the released approach. It adds a
source exit reaching terminal DST244, which also feeds JOINT55.

That direct source exit is a synthetic feasibility witness, not an assertion
that this exact jump exists on the real chip. The reduced test isolates the
shared-terminal restriction; whole-chip reachability must be checked separately
before attributing the complete Example 1 failure to it. The follow-up replay
below found a different, earlier rejoin into the retained SRC and imports it
as a separate case.

## Real bypass found by the follow-up replay

The unchanged Example 1 replay reproduced the same run-4 binding and gap.
Normal source repair exhausted 32,768 states. The read-only reachability audit
found the following source-to-sink path when allowing replacement of
same-source incoming arcs:

```text
(58,50)/LOCAL156 -> SRC3075 -> (50,50)/DST3080
 -> SRC3075 -> (41,50)/DST3080 -> LOCAL184
 -> SRC818 -> (45,49)/DST977
 -> SRC161 -> (45,43)/DST161
 -> SRC977 -> (50,39)/DST977
 -> SRC545 -> (53,40)/DST545 -> JOINT55 -> LOCAL63
```

The existing disconnected tail feeds `(50,39)/SRC545` from **DST737**.
The discovered bypass feeds it from **DST977**. All intermediate resources
are free or owned by the same source. The terminal JOINT input itself does
not change on this path. The repair's `allowed()` predicate rejects the
different incoming arc to the retained SRC; its private-terminal exception
cannot apply to a SRC on the preceding Tile.

`retained_src_rejoin` imports every numeric arc and coordinate of this witness
and the last complete owner route, translated by `(-41,-38)`. It preserves the
two shared-terminal bindings and a completed sibling owning the witness's
first hop. It reduces unrelated congestion to a committed foreign Grounding
blocking the exact gap. It does not claim to replay every route on the chip.

Both full-chip audits reached their one-million-state bound. The relaxed audit
nevertheless produced the complete positive witness above. Failure to find
the old gap endpoint is not evidence that it is globally unreachable. In the
small imported fixture, neither audit hits a limit, and repair still rejects
the source-to-sink witness. This separates the rejection from search budget.

## Cases

| CTest suffix | Difference | Required result |
| --- | --- | --- |
| `shared_terminal_gap` | Two interrupted bindings, replacement terminal input | Repair both coherently |
| `shared_terminal_private` | Same topology, no shared replica | Repair the owner |
| `shared_terminal_exact` | Bypass restores the original gap endpoint | Repair both, preserving the terminal input |
| `shared_terminal_blocked` | No bypass | Reject without changing routes |
| `retained_src_rejoin` | Imported bypass enters the disconnected tail's SRC through a new DST | Repair the shared tail coherently |
| `retained_src_exact` | Imported owner route, added bypass to its original gap endpoint | Repair both bindings |
| `retained_src_protected` | An interrupted branch to another destination also uses the SRC | Reject without changing any binding |

Run all seven with:

```sh
cmake --build build --target grounding_preemption -j2
ctest --test-dir build -R '^fpga.fanout_(shared_terminal_|retained_src_)' --output-on-failure
```

## Reproduction before a production fix

The shared-terminal case fails normally (not `WILL_FAIL`):

```text
physical_reachable=1 preserving_inputs=0 repaired=0
shared-terminal gap repair rejected a reachable replacement input
```

All four controls pass. The imported `retained_src_rejoin` case also fails
normally, with `physical_reachable=1 preserving_inputs=0 repaired=0`:
strict reachability visits 28 states and relaxed reachability visits 32.
Neither exhausts its limit, and every congestion audit remains clean.

For the initial terminal-JOINT case, strict source reachability visits five states,
rejecting the new incoming arc at the retained shared terminal. Permitting
replacement of same-source input arcs reaches LOCAL63 in seven states:

```text
(9,12)/LOCAL300 -> SRC706
    -> (4,2)/DST244 -> JOINT55 -> LOCAL63
```

This is not search-budget exhaustion or an unowned busy bit. The source
repair's `allowed()` predicate calls `canReplacePrivateTerminalInput()` when
an occupied same-source terminal would acquire a different predecessor. That
guard explicitly rejects another binding's use, including a non-owning
replica. Simply removing it would be unsafe: the replica would retain its
obsolete incoming arc. The eventual fix must update the affected interrupted
bindings together and preserve unrelated completed users. The imported
retained-SRC case additionally requires coordinated replacement at the earlier
disconnected-tail junction, not just at the final JOINT/LOCAL.

The test checks the precise gap boundaries, physical reachability, completion,
ownership before/after repair and unchanged foreign Grounding. On a failed
repair it also verifies that all retained route fragments are unchanged.

## Coordinated replacement fix

Source repair now recognizes final-tail nodes used exclusively by interrupted
bindings sharing its driver and physical sink. It permits a provisional new
incoming arc there, then replaces all those bindings together after validating
the complete path. Completed routes, live prefixes and other destinations are
excluded. Existing prefix ownership is promoted outside the replacement group,
and all replacements are registered before releasing old leases.

The two formerly failing cases now pass, as do the controls. The older private
Grounding regression also requires interrupted same-input replicas and RTL
aliases to complete together; it still rejects foreign owners. Strict
`preserve_inputs` reachability remains a diagnostic of the original stored
arcs, not a simulation of this coordinated edit.
