# remote_local_reservation

Purpose: Distinguish a sink-owned packed LOCAL from foreign LOCAL transit.

Fixture: abstract numeric CB nodes and resource pins.
The test invokes the production routing entry used by Moving.

Focused checks:
A placed input reserves its terminal LOCAL.
A foreign route cannot transit through that LOCAL.
The sink terminal callback may accept its own LOCAL.
An unrelated LOCAL remains available for ordinary transit.
Remote LOCAL continuation uses numeric resolved connectivity.
Accepted terminal fragments retain the input pin identity.

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
