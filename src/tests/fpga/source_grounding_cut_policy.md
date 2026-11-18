# source_grounding_cut_policy

Purpose: enforce Grounding cut policy after tracing a free source corridor.
An occupied terminal arrival must be cut only when the caller permits it.
Implementation: `source_grounding_cut_policy_regression`.
Run: `ctest --test-dir build -R '^fpga.source_grounding_cut_policy$' --output-on-failure`.

The fixture runs with corridor lengths of eight, twenty-four and forty-eight.
A completed sibling preserves the source Takeoff and a separate input branch.
The new binding follows a free corridor toward another sink.
A real foreign transit owner occupies the final DST500.
The sink LOCAL is reachable from that arrival when the obstruction is removed.
Each corridor runs with terminal cuts refused and with terminal cuts allowed.
The callback accepts only the expected terminal Tile, type and node.
Cut previews use actual registered victims and affected route fragments.

Immediate cancellation must leave routes, leases and deadend bits unchanged.
A four-label free-search cap must fail without installing a binding.
That capped call must preserve both existing deadend masks.
The normal call must succeed exactly when the terminal cut is permitted.
A successful call must apply one cut and install a complete connection.
A refused cut must leave route ownership unchanged.
Repair must not create deadends during either failure or completion.
The accepted path must contain corridor length minus one physical jumps.
Ownership audits check both the completed sibling and the displaced transit.
Diagnostic output reports the effective free-search cap.
History: formerly named `source_frontier_reuse` for a removed reuse mechanism.
The obsolete reuse assertion was removed; no cache was reintroduced.
History: completed-path validation no longer rejects persistent SRC deadends.
History: learning marks is now exclusive to TraceRouteForward/Backward.
