# moving_local_bridge

Purpose: Check direct and single LOCAL bridge selection in forward tracing.

Fixture: abstract numeric CB nodes and resource pins.
The test invokes the production routing entry used by Moving.

Focused checks:
A direct path keeps priority over a LOCAL bridge.
A free LOCAL can connect to a SRC through one typed arc.
Occupied and reserved LOCALs cannot be used for transit.
A throwing sink probe unwinds all leases.
A short expansion budget stops at a physical hop.
Prepared commit rejects stale connectivity and busy landings.

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
