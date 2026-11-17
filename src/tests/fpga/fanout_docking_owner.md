# fanout_docking_owner

Purpose: distinguish same-Tile transit from protected endpoint Grounding.
The production Fanouts/Docking route must cross eligible foreign transit.
Implementation: `fanout_docking_owner.cpp`, using an abstract 10-by-10 fabric.
Run: `ctest --test-dir build -R '^fpga.fanout_docking_owner$' --output-on-failure`.

A completed trunk provides the starting DST at (7,7).
A free departure cannot reach the requested sink LOCAL at (3,6).
The useful six-jump approach passes through five independent route owners.
The contested owner enters (7,3)/DST26 and traverses SRC23 to DST23 there.
It then continues toward its own sink, whose cell is on another Tile.
This same-Tile hop has a SRC and landing DST, without a terminal LOCAL.
Two retained gaps are created through production partial unrouting.
Initial routes are registered and leased through production functions.
The fixture never invents busy masks to represent those owners.

Physical-owner lookup must recognize the incoming DST, SRC and landing.
Transit-owner lookup must recognize that same fabric continuation.
Partial-cut preview must preserve the victim's protected endpoints.
The request must complete through the useful approach.
Every Tile receives a congestion audit before and after the request.
Failed attempts must preserve physical leases and fixed endpoint positions.
Related free, ordinary-transit and removed-owner controls require completion.
The genuine-Grounding control must reject preemption without changing leases.
History: a captured owner was excluded solely because its hop stayed in one Tile.
The resulting `endpoint_owner` rejection blocked an otherwise legal connection.
Classification and cut planning now identify transit by numeric hop resources.
Takeoff and terminal Grounding retain their protection.
History: ownership and partial-cut implementations moved from Wire to Docking.
