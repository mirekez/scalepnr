# fanout_retained_suffix

Purpose: reconnect a reached, physically intact retained suffix.
The source trace must complete it without rediscovering every suffix node.
Implementation: `fanout_retained_suffix_regression` in `fanout_gap_replay.cpp`.
Run: `ctest --test-dir build -R '^fpga.fanout_retained_suffix$' --output-on-failure`.

The main fixture retains an interrupted route and a complete sibling.
The exact GAP frontier has eight exits blocked by registered foreign owners.
A nine-hop alternative reaches (16,15)/DST978 at the suffix start.
The retained continuation has four more physical hops to the sink.
Eight irrelevant landings each expose a full mask of terminal-less LOCALs.
Those branches apply search pressure without inventing occupied resources.
The completed replacement must contain thirteen physical jumps.
The sibling's physical path and all foreign routes must remain unchanged.
The trace must identify the retained-suffix join below the default state cap.

A missing final arc must prevent completion despite stored route fragments.
The broken-topology control uses an independent fixture and retains deadends.
A twelve-hop bound must reject the thirteen-hop source-to-sink route.
Immediate cancellation must leave routing unchanged.
Live leases must reject a cyclic suffix without scanning the active prefix.
Its alternate loop-free source-to-sink route must still complete.
Two and three visits to one Tile are legal through distinct routing nodes.
The Tile re-entry cases must complete their five-hop paths.
A missing second-JOINT connection or packed JOINT reservation must block them.
Ownership audits verify every retained and newly routed resource.
History: the regression originally exposed rediscovery of an intact suffix.
An obsolete third-Tile-visit rejection was corrected to require completion.
Suffix checks now lease nodes in place; independent controls retain learned deadends.
