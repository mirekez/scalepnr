# bounded_deadend

Purpose: Directional mask filtering in bounded Grounding.
Run: ctest --test-dir build -R "^fpga.bounded_deadend$" --output-on-failure.

Fixture:
Forward and reverse frontiers meet through real numeric connectivity.
The tests use abstract numeric routing-node identities.

Focused checks:
Reverse search ignores SRC marks and uses DST marks.
Physical occupancy remains independent of persistent search deadends.
A failed attempt must release only its own newly acquired leases.
Unrelated occupied resources must remain unchanged.
Repeated attempts must not inherit a rejected placement decision.

Directional contract:
Forward routing reads SRC deadends.
Reverse routing reads DST deadends.
Only TraceRouteForward and TraceRouteBackward write the respective masks.
A jump may finish at its eligible endpoint without another outgoing jump.
Completed-path ownership checks do not recheck search deadends.

Failure evidence:
Lost valid completion indicates a wrong direction or endpoint filter.
Changed unrelated masks indicate an incorrect lease restoration.

History: updated for symmetric persistent SRC and DST deadends.
History: full Docking remains a reader of both directional masks.
Scope: focused regression; excludes the generated 50K routing run.
