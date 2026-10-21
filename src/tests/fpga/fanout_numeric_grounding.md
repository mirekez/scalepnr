# Numeric prefix reaches an occupied but preemptible terminal

After the internal-LOCAL fix, Example 1 run 5 completed signal 206 and
advanced to a different failed connection, signal 1869, from `(63,11)/LOCAL156`
to `(25,12)/LOCAL63`. The destination resource is `(24,12)`. Both ordinary
branch search and the new free-only numeric fallback failed; neither a stage
deadline nor stagnation caused the failure.

The destination and twelve predecessor Tile audits were clean. All sixteen
terminal DST alternatives had real transit owners. Whole-source free-only
reachability exhausted 966915 states without reaching the sink, so raising
the free search budget cannot solve this terminal obstruction.

The bounded numeric search nevertheless reached `(25,11)/DST261` through a
17-hop prefix using the existing source SRC261. The recorded prefix contains
four internal LOCAL bridges and a same-Tile SRC23-to-DST23 continuation.
It can finish through:

`(25,11)/DST261 -> SRC17 -> (25,12)/DST20 -> JOINT55 -> LOCAL63`.

SRC17 and DST20 are occupied by one completed transit route; the JOINT and
sink LOCAL are free. The new-task wrapper omitted the existing Grounding
preemption callback, so it discarded this approach without checking the cut.

`fpga.fanout_numeric_grounding` imports every numeric prefix arc, coordinates,
the completed four-fragment source trunk and the exact conflicting transit
jump. Only unrelated occupied alternatives/branches and the victim's remote
prefix/suffix are omitted. Names are abstract. It failed before the fix while
the protected-Takeoff and disconnected-prefix controls passed.

The fix connects unbound numeric completion to the existing preview/commit
policy, after free-only searches fail. It validates the whole projected path
before cutting anything and installs no temporary/incomplete binding. The
test also repairs the preserved victim via a separate bypass, checks both
routes complete, and audits ownership after suffix teardown. Negative controls
must leave the victim complete and create no suffix.
