# moving_branch_takeoff_preemption

Purpose: Try free joint alternatives before cutting foreign transit.

Fixture: abstract numeric CB nodes and resource pins.
The test invokes the production routing entry used by Moving.

Focused checks:
The retained source trunk supplies an owned branch DST.
A foreign route occupies one candidate transit resource.
A parallel free JOINT can reach the same SRC.
One mode has a direct JOINT to SRC path.
Another mode has a two-JOINT path requiring bounded Docking.
The completed route records the selected JOINT identities.

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
