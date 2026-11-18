# moving_forward_memory

Purpose: Bound stack and temporary memory during deep recursive tracing.

Fixture: abstract numeric CB nodes and resource pins.
The test invokes the production routing entry used by Moving.

Focused checks:
A cyclic grid provides many distinct numeric arrivals.
The trace may revisit a Tile through a free DST.
Each recursive call handles one physical CB hop.
The child process has a fixed address-space growth allowance.
A finite expansion budget ends the search.
All leases return to their original masks after failure.

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
