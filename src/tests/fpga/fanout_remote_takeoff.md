# Repair a remote sink through a new Takeoff

Example 1, run 4, failed after two preemptions reduced a completed route from
eleven to nine fragments. Its source was `(54,37)/LOCAL163`, the retained gap
was `(54,38)/DST24 -> (57,41)/DST547`, and its sink was `(57,47)/LOCAL88`.
The gap-frontier audits checked 185 and 174 occupied nodes respectively with
no missing owners, leases or registrations. A separate sink-area audit was
also clean. These are local audits, not a whole-device claim.

Free source search exhausted 66 states in two Tiles, with no budget rejection.
It never reached the five-Tile Grounding window. A read-only Takeoff probe
found this complete six-hop path, validating all proposed cuts together:

```
(54,37) LOCAL163 -> SRC3376 -> (50,41) DST3376 -> LOCAL183 -> SRC819
 -> (54,45) DST819 -> SRC3619 -> (53,46) DST3619 -> SRC19
 -> (53,47) DST22 -> SRC547 -> (54,48) DST547 -> SRC262
 -> (57,47) DST260 -> JOINT51 -> JOINT36 -> LOCAL88
```

Only the first SRC and its landing DST need preemption. Both belong to one
27-fragment transit binding. The old repair supported free departure and
preemption-assisted Grounding, but not this Takeoff outside the sink window.

`fpga.fanout_remote_takeoff` keeps the nine requester fragments, the entire
27-fragment transit binding, and the witnessed replacement. Seven unrelated
owner bindings and other blocked alternatives are omitted; shared-prefix
ownership is normalized for those omitted replicas. A synthetic newer
Takeoff blocks the old prefix. All fixture names are abstract and every busy
resource has a registered route owner. This is not a full-chip replay.

The unfixed test fails at its intended missing-Takeoff assertion. The fixed
case checks six-hop continuity, intact newer Takeoff, retained victim
endpoints, exactly one requeued victim, and clean occupancy. Disabled
preemption, a protected owner, and a broken intermediate connection must
leave all leases unchanged. `fpga.fanout_remote_takeoff_blocked` additionally
occupies the second hop outside Grounding with transit: repair must not
misclassify an intermediate LOCAL bridge as a new source Takeoff.

The fallback runs only after existing repair searches fail. It previews
eligible cuts at the initial source departure and its first landing, requires
free transit thereafter, and permits bounded completion in the existing
Grounding window. Node-cycle/Tile-revisit, state, total-hop and cancellation
limits remain enforced. Full-path validation and atomic commit precede any
victim mutation; no unfinished replacement is installed.

Validation: the positive test failed before the production change at
`BUG: repair misses source Takeoff outside the Grounding window`; both new
tests pass with the fix. The complete 141-test suite, including all eleven
puzzles, passed in 2,148.28 seconds without relaxed requirements or limits.
End-to-end results must be reported separately; these tests alone do not
establish that Example 1 finishes.

The subsequent real replay committed this six-hop replacement: 2,789 search
states, two cuts, 158.725 ms search and 1.935 ms commit. The route history
records the repaired binding as complete. Run 4 then advanced to pass 23,
with 193 tasks remaining, before the unchanged 600-second deadline expired
on another binding. Profiling identified exact-gap floods as the dominant
remaining cost; see `fanout_gap_budget.md` for the separate regression/change.
