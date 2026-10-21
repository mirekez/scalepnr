# Remote LOCAL continuation lost behind terminal branches

Example 1 run 6 failed signal 166890, from register 7598/Q to cell 4178/I1.
The source CB was `(45,13)/LOCAL141`; the input was `(37,36)/LOCAL83`.
The source and sink resource Tiles were `(46,13)/128` and `(38,36)/131`.
The reported generic-search stop was `(49,1)/DST3537`. It was neither a
stage timeout nor a stagnation termination.

All 1,580 captured blocked-frontier ownership snapshots were clean: no
orphan leases, missing leases/registrations, stale registrations, or conflicting
drivers. These are the recorded frontier Tiles, not a whole-device audit;
the forward search never reached the sink Tile.

The numeric free search allocated 32,768 states (29,576 unique), expanded
32,099 states, and rejected 1,685 insertions at its budget. Its closest
reached position was `(41,28)`, eight Tiles from the sink in Chebyshev distance.
Consequently the five-Tile Grounding completion had **zero** starting states.
The Takeoff fallback also exhausted 32,768 states.

An independent input-preserving audit reached the input after 2,771,556 states
without hitting its diagnostic limit. Its 80-node, 62-Tile-hop witness is
recorded in `basic_remote_local_path.txt`. Every node on it was free. The
bounded search reached the witness through `(66,11)/SRC816`, but could not
allocate its next landing `(70,15)/DST816`.

## Reproduction and correction

`basic_remote_local_frontier.txt` contains 92,212 unique allowed arcs from
the failed search plus 67 missing witness arcs. Each row is
`x y kind id next_x next_y next_kind next_id`, with LOCAL=0, JOINT=1,
SRC=3, DST=4. All textual node names are abstract. Resource Tiles and positions
are normalized, CB type IDs are unique per fixture Tile, and unavailable
foreign-owner alternatives are omitted rather than represented by unowned bits.
This is a reduced admitted frontier, not the entire device topology.

The normal-budget test failed before the correction. The 65,536-state control
passed; removing the last LOCAL-to-input edge failed safely without publishing
a route. All three cases also check cancellation, fixed endpoints, occupancy
audits and teardown. The positive result must not exceed the witnessed 62 hops.

The correction extends structural leaf rejection through short JOINT/LOCAL
chains. Eight stack entries bound this proof; an unresolved deeper branch
remains eligible. Cycles with no exit are terminal, while source/sink,
fabric exits and usable dedicated LOCAL continuations remain searchable.
No persistent cache, net-name rule, increased routing budget or congestion
deadend assumption is introduced. Unrelated occupied nodes are rejected
before this lookahead; provisional cuts and same-source nodes retain their
ordinary ownership/input checks.

A separate check against the **real loaded topology**, not just this reduced
fixture, found 3,329 provably terminal JOINTs and 1,426 provably terminal LOCALs
among the unique nodes in the failed frontier. The initial experiment passed
all three new cases and found the 62-hop path.

Final validation passed **180/180** CTest tests, including all eleven puzzles,
in 2,347.22 seconds. The 50K-cell forest passed in 265.87 seconds under its
unchanged 300-second limit. The complete abstract witness search took
685.550 ms in the debug regression build, including opt-in decision logging.
An unrelated build was active during the suite; these are validation times,
not an isolated before/after performance benchmark.

## End-to-end caveat

The real witness traverses a dedicated regional distribution network. Its
edge `(78,26)/LOCAL337 -> LOCAL273` is a buffer pseudo-connection, not an
ordinary configurable switch. The database and reference exporter require
buffer-enable configuration for this use. The exporter previously silently
dropped the pseudo-feature without adding those bits. The separate regression
`tests/prjxray/db2fasm_test.py` reproduces this with abstract routing-node names;
the corrected adapter uses the buffer site's I/O metadata to emit its enable
configuration. Ordinary fixed wires do not enable unrelated buffers, and
missing configuration features produce warnings. The related global buffer
case also has coverage: a placed buffer must be enabled even when neither of
its separately routed I/O nets contains its internal pseudo-connection. Its
concrete site selects the configuration; invalid sites produce warnings.
All 62 backend script tests pass. Checking the real database resolves 48
regional and 32 global buffer pseudo-connections, and emits both required
regional bits for this witness. Abstract routing reachability
and this export check still do **not** establish a completed Example 1
bitstream; the full routing/export flow must finish first.
