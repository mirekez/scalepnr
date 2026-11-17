# source_takeoff_work_open

Purpose: complete the Takeoff work fixture when its sink connection is present.
The source trace must preserve cut policy while avoiding ancestor scans.
Implementation: `source_takeoff_work_regression(true)`.
Run: `ctest --test-dir build -R '^fpga.source_takeoff_work_open$' --output-on-failure`.

A registered transit route owns the requester's first SRC and landing DST.
The cut callback accepts only those two resources of that owner.
A 128-lane reconvergent corridor extends beyond the occupied departure.
An added final jump connects that corridor to the requested sink LOCAL.
The physical completion has twenty-six jumps.
The same public search object serves the negative controls and positive request.
All node names and routing owners are abstract fixture data.

Refusing the cut must leave the transit owner unchanged.
Immediate cancellation must preserve all routes and leases.
A sixty-four-label bound must fail and report budget exhaustion.
The larger request uses a 97,284-label budget and a one-hundred-hop bound.
That request must install one complete route and apply exactly one cut.
The completed route must follow a continuous twenty-six-hop physical path.
The callback must never select a different foreign owner.
Cut previews must precede the single accepted mutation.
The source-probe parent-walk counter must remain exactly zero.
Node availability comes from leases held by the active recursive calls.
Unwinding preserves pre-existing occupation while releasing each call's own lease.
Owner audits reject orphan bits and missing route registrations.
History: the open variant checks policy changes across repeated public requests.
History: ancestor-walk assertions now require zero scans.
Takeoff retry lives in Docking and uses Wire's shared angle ordering.
