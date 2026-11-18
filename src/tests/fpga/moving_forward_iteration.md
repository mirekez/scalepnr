# moving_forward_iteration

Purpose: Keep one recursive frame per physical CB hop through a LOCAL relay.

Fixture: abstract numeric CB nodes and resource pins.
The test invokes the production routing entry used by Moving.

Focused checks:
A busy direct landing is rejected in the root frame.
The same frame then selects a free LOCAL relay.
The relay reaches a different Tile without restarting the anchor.
The sink probe sees only the selected relay lease.
A failed direct alternative leaves no occupancy.
Frame and edge counts follow physical jumps.

Failure interpretation:
A missing route means a legal numeric connection was skipped.
A wrong route means ownership or path identity was lost.
An extra lease means recursive unwinding changed live state.

Routing contract:
SRC and DST deadend masks remain persistent and directional.
LOCAL and JOINT checks use loaded numeric connectivity.
Angle tracing resumes its local cursor after a failed child.
Full internal chains belong to bounded Docking.

The test does not depend on vendor node names.
The test does not run the generated 50K design.

History: documented the physical-hop and full-search boundary.
