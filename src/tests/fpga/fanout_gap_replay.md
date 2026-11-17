# fanout_gap_replay

Purpose: reproduce a trapped GAP, then verify a captured source bypass.
The replay retains numeric connectivity while generating abstract node names.
Implementation: `fanout_gap_replay_regression` in `fanout_gap_replay.cpp`.
Run: `ctest --test-dir build -R '^fpga.fanout_gap_replay$' --output-on-failure`.

Production transit cuts interrupt an initially complete eighteen-fragment route.
The resulting binding has two GAPs and fifteen retained fragments.
Its first GAP starts at (16,28)/DST242.
The frontier has two reachable states, twenty busy SRCs and three busy landings.
Every busy node has registered foreign owners; no naked busy bits are injected.
A read-only whole-source traversal exhausts thirty states in the reduced topology.
Default and doubled routing budgets must both fail in that topology.
The production Fanouts task must report the actual unresolved GAP boundary.
Failed searches must preserve routes and ownership while retaining proved deadends.

The imported bypass adds independently captured numeric arcs.
One path reaches the first retained GAP end through intermediate LOCAL165.
A separate ten-hop path reaches the sink through intermediate LOCAL178.
Independent reachability checks verify both paths and their required LOCAL arcs.
Missing-bridge and packed-bridge controls must reject completion.
Each topology control uses its own fixture, keeping any learned deadends.
Cancellation, one-state and nine-hop bounds must reject the complete bypass.
The successful production Fanouts task must install a continuous ten-hop route.
Foreign routes and the independently interrupted sibling must remain unchanged.
LOCAL178 must be leased as a routing wire, not as a cell pin.
Repeated transit cuts exercise SRC, LOCAL, incoming DST and landing DST repair.
History: the reduced capture originally omitted the verified source bypass.
Independent controls now prevent restored topology inheriting old deadends.
