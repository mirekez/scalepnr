# source_takeoff_work

Purpose: bound Takeoff tracing work behind an occupied source exit.
The disconnected case must exhaust its available corridor without leaking leases.
Implementation: `source_takeoff_work_regression(false)`.
Run: `ctest --test-dir build -R '^fpga.source_takeoff_work$' --output-on-failure`.

A real transit route owns the requester's first SRC and its landing DST.
A cut callback permits only those two resources of that registered owner.
Beyond the obstruction, 128 lanes traverse twenty-four additional Tile layers.
Each incoming lane connects to thirty-two forward exits.
The last landings do not connect back to the requested sink.
A separate free terminal approach has no connection to the source corridor.
Node and owner names are abstract generated patterns.

The requester must fail without installing any route fragments.
The original transit owner must remain complete and unchanged.
At least two cut previews must exercise the occupied source departure.
No preview may apply a cut or leave a live lease behind.
The test expects 97,283 labels and 184,579 edges from its explicit lane fan-in.
Its 97,284-label budget covers SRC choices even when their landings reconverge.
The source-probe parent-walk counter must be exactly zero.
Recursive node leases must reject reused nodes through availability checks.
Active cut resources are protected from selection for another cut.
Existing foreign occupation must survive every failed recursive return.
Owner audits check every lease and registration after the request.
The test uses deterministic operation counts rather than elapsed-time thresholds.
History: this case reproduced excessive ancestor walks in Takeoff completion.
Its previous assertion merely bounded the number of ancestor steps.
History: Takeoff retry and its successor traversal moved entirely into Docking.
