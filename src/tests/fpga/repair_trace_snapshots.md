# repair_trace_snapshots
Purpose: keep routing diagnostics complete without duplicate state dumps.
Use an abstract converging numeric source-search graph.
Sixty-four source exits converge through sixty-four downstream exits.
The requested sink is disconnected from that component.
Run an untraced control against the production source search.
The control must fail without leaving a route or orphan lease.
Construct the traced control from the same initial physical state.
Do not reuse deadends learned by the earlier control.
Enable the real route congestion diagnostic stream.
The traced search must report the same disconnected outcome.
Keep individual repair decision records available for inspection.
Unchanged Tile state must not produce repeated snapshot references.
The count of full snapshots must follow touched state, not edge count.
Publishing an owner can preserve the exact same busy masks.
Such publication must change the route mutation revision.
A snapshot after that change must refresh the owner information.
Persistent deadends are also part of a diagnostic snapshot.
Use an initially clear diagnostic bit for that state-change check.
Setting that bit must produce a new full snapshot.
The check must not erase a previously learned deadend.
Audit every real occupied node after the traced search.
Also exercise the deferred deadline-diagnostic mode.
Two unchanged deferred snapshots must produce one state record.
Deferred mode must not emit duplicate snapshot references.
An owner publication must refresh its deferred snapshot too.
Deferred owner auditing must not hide publication changes.
Diagnostic mode must not change routing success or ownership.
History: introduced converging-edge snapshot suppression checks.
History: isolated the controls because searches now retain deadends.
