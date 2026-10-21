# Complete free path with an internal LOCAL bridge

The run-5 Example 1 snapshot `fanout_206_reach.tYMOuZT0` has a free
source-to-sink path, not merely a free terminal suffix. Independent numeric
reachability reaches the sink after 497,176 diagnostic states, without a
state-limit hit or any preemption. The resulting path has ten Tile hops:

`(63,6) -> (58,2) -> (50,2) -> (41,2) -> (32,2) -> (26,6) -> (26,12)
-> (26,18) -> (26,24) -> (32,29) -> (36,29)`.

At `(32,2)` it requires `DST3077 -> LOCAL165 -> SRC3376`, outside the
five-Tile Grounding window. Every node of this witness was unleased in the
audit. The fast forward search only enumerates DST-to-SRC paths through up
to two JOINTs. In the real snapshot it also exhausts its 200-state quota
under the relevant source exit before reaching the bridge.

`fpga.fanout_transit_local` imports the completed six-fragment trunk and the
entire witness with the recorded coordinates and numeric connectivity. All
names are synthetic. Unrelated congestion and misleading forward choices
are omitted to isolate the unsupported LOCAL transition. Before the fix the
independent audit succeeds but production Fanouts fails. Replacing only the
bridge by a direct DST-to-SRC arc passes; removing the bridge fails safely.
These controls distinguish a fixture error from missing bridge support.

The bounded numeric source search already used for interrupted-route repair
is reused only after ordinary fanout alternatives fail. New-task mode searches
free paths first, with no detached-tail reparenting, temporary live bindings,
or incomplete route installation. It validates the whole path before leasing
and attaching it. Existing successful fast searches are unchanged. Cancellation,
original trunk survival, fixed endpoints, and complete node ownership are
checked by the regression. This reduced test does not by itself prove that
the real congested snapshot fits the numeric search budget.

The `transit_local_reuse` variant supplies a 13-hop route branching from the
existing trunk as well as the shorter fresh Takeoff. It exposed an initial
fallback policy regression: shortest-source search picked the new Takeoff.
New-task completion now first searches using an already owned initial SRC;
only failure of that bounded attempt permits a fresh Takeoff. The test checks
all 13 hops, no new source exit, and teardown preserving the original trunk.

The later `fanout_numeric_grounding` regression covers bounded terminal
preemption after free alternatives fail; this free witness still needs no cut.
