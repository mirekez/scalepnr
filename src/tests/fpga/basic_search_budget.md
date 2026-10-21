# Basic search exhausts its frontier on unrelated terminal leaves

Example 1 run 6 failed signal 1601 from `(53,10)/LOCAL142` to
`(15,7)/LOCAL127`. The selected task's 1,991 congestion audits were clean.
The destination's free approaches alone did not establish a full route.

An independent, input-preserving, free-only audit found a **24-hop** path
after discovering 1,595,286 states, without hitting its four-million-state
diagnostic bound. Every node on that witness was free. The path includes
`(26,4)/DST739 -> LOCAL128 -> LOCAL184 -> SRC98` and a legal second arrival
at `(11,5)`. It needs no preemption or movement.

The production numeric search reached `(25,10)/DST3841` at hop 16, cost 98,
but rejected its free continuation `SRC3076` because its 32,768-state budget
was exhausted. Of the allocated states, 16,837 were LOCALs. A LOCAL with no
outgoing physical arc, other than the required sink, cannot finish a route
and should not consume this bounded frontier.

`basic_search_frontier.txt` contains 35,308 distinct physical arcs:
accepted parent-to-child arcs and budget-rejected arcs from that numeric
search, plus all arcs of the independent complete witness. Each line is:

`from_x from_y from_kind from_id to_x to_y to_kind to_id`

Kinds use `CBNodeNameType`: LOCAL=0, JOINT=1, SRC=3, DST=4. Coordinates and
numeric topology are retained; names are generated abstractly by the fixture.
These are routing-CB coordinates. Endpoint resource Tiles are colocated with
their CBs in the isolated fixture; the real resources were `(52,10)` and
`(14,7)`. This test exercises routing, not placement or resource-to-CB mapping.
This is a reduced **free-edge** snapshot, not a full chip or a claim that
all omitted alternatives were occupied. It contains no invented background
load, arbitrary busy bits, completed seed route, or preemptible victim.

Before the fix:

- `fpga.basic_search_budget`: fails to complete at the production 32,768 limit.
- `fpga.basic_search_budget_control`: completes the same 24-hop witness at
  65,536 states, establishing reachability independently of the failure.
- `fpga.basic_search_budget_blocked`: removing the sink's only terminal arc
  leaves no route and no speculative bindings or leases.

The fix checks numeric outgoing topology before admitting an unrelated LOCAL.
It does **not** reject all LOCALs or all input pins, consult deadend heuristics,
increase the state/hop bounds, or introduce persistent caches. Intra-Tile
LOCAL-to-LOCAL, LOCAL-to-SRC/JOINT, and dedicated cross-Tile LOCAL connections
are preserved. The independent audit is deliberately unchanged.

Positive cases require the 24-hop path and its two-LOCAL bridge, fixed endpoint
coordinates, exactly one complete binding, and clean occupancy/registration
audits before and after teardown. Cancellation and disconnected controls must
leave no route. The higher-budget case is a test control, not a new router
setting. Real-chip progress still requires replaying Example 1 after the full
regression suite.
