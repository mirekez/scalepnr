# Free continuation beyond the bounded frontier

Example 1's unchanged-design replay in `auto_failure_retry.HXygepPE`
completed runs 1–5, then failed run 6 Basic routing on connection 165390:
register 6516/Q at resource (74,1) to cell 17465/I5 at resource (13,13).
The actual CB endpoints were (75,1)/LOCAL142 and (12,13)/LOCAL84.
The reported generic-search stop (57,5)/DST3616 was neither endpoint.
Neither the stage deadline nor stagnation caused the failure.

The actual source, sink, and generic-stop ownership audits were clean:
no unowned or missing leases, missing/stale registrations, or conflicting
drivers. The read-only numeric completion probe found a completely free
52-node, 21-Tile-hop path, without requesting any preemption. In the full
device it allocated 111,911 states and expanded 104,073. Both 65,536-state
priorities had exhausted their budgets without reaching the sink.
The independent input-preserving reachability audit also reached the sink.

The required detour includes (22,12)/DST3845 → LOCAL183 → SRC1031,
then moves away from the sink through (32,12), (32,14), and (32,20).
The last approach is (5,14)/SRC3078 → (11,14)/DST1030 → SRC738 →
(12,13)/DST738 → LOCAL84. A free terminal alone was not the proof:
the diagnostic validated the entire source-to-input path.

`basic_free_detour_frontier.txt` contains the union of both captured
65K admitted frontiers and the missing witness arcs: 342,117 unique arcs.
This is a reduced frontier, not the entire FPGA topology.
`basic_free_detour_sources.txt` preserves 29,918 raw physical SRC mappings
and SRC/JOINT alternatives, including unavailable landings that affect priority.
All textual node, Tile, net, and cell instance names in the fixture are abstract.
Resource positions are normalized to the corresponding CB coordinates.

Every omitted busy/unexpanded outlet is represented by a separately routed
foreign net, rather than an unowned occupancy bit. Those nets also preserve
the structural productivity of captured LOCAL/JOINT branches. Ownership
is checked before and after the attempt and after requester teardown.
The fixture does not move endpoints or feed its witness to the router.

Regression cases:

- `fpga.basic_free_detour`: enters Basic's actual task routine. Before the
  fix, its original 32K and 64K attempts fail with the captured budget miss.
- `fpga.basic_free_detour_budget`: an explicit 128K low-level control finds
  the complete 21-hop free route (72,301 states in the reduced fixture).
- `fpga.basic_free_detour_limited`: a one-state low-level attempt must remain
  bounded and return failure without bindings, cuts, or leaked leases.
  Do not require a 64K attempt to fail forever: a future faster search is
  allowed to solve this connection inside that budget.
- `fpga.basic_free_detour_disconnected`: removes the final physical
  connection; a larger budget must not invent a route or damage owners.

The caller adds one final 128K recovery only after all original attempts
for every pin pair have failed. Earlier successful choices remain untouched.
The low-level per-attempt bound, 200-hop bound, preemption policy, and stage
deadline are unchanged. Real Example 1 revalidation is still required.

Before the fix, the actual Basic caller failed while all three controls
passed (164.96 seconds, /tmp/scalepnr-free-detour-caller-red.log).
With the bounded caller recovery, all four cases passed (255.43 seconds
while the full rebuild and unrelated workloads were active,
/tmp/scalepnr-free-detour-green.log). Routine runs do not regenerate the
large textual frontier; the captured fixture and independent owner audits
provide that coverage directly.
After those runs, the explicit-limit control was changed from 64K to one
state for the reason above. That final control passed independently in
43.62 seconds (/tmp/scalepnr-free-detour-limited-final.log and .xml);
full-suite validation remains required.
