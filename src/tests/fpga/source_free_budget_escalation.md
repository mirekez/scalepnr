# source_free_budget_escalation

Purpose: honor the caller's full source budget after a smaller fast cap fails.
The free route must complete without requiring a transit cut.
Implementation: `source_free_budget_escalation_regression`.
Run: `ctest --test-dir build -R '^fpga.source_free_budget_escalation$' --output-on-failure`.

Corridor lengths are eight, twenty-four and forty-eight Tiles.
Fast source caps are four and sixteen labels.
A completed sibling keeps the initial source trunk occupied and reusable.
The free corridor crosses an internal LOCAL bridge.
Three terminal JOINTs connect its last arrival to the requested sink LOCAL.
A registered foreign route owns a separate arrival near that sink.
Variants omit preemption or offer a callback that refuses every cut.
Connected and disconnected variants each construct an independent fixture.

A request limited to the small cap must fail without installing a binding.
Proved deadends may remain, but routes and existing owners must not change.
Immediate cancellation must preserve the full state after the limited attempt.
Removing the terminal arc must prevent completion even with the full budget.
The disconnected fixture retains its deadend marks and is then discarded.
The connected fixture must complete within the explicit 4,096-label budget.
Its route must contain corridor length minus one physical jumps.
The completed sibling and foreign route history must remain unchanged.
The diagnostic must show the small capped attempt before full completion.
Unrouting must release the requester without damaging foreign ownership.
History: the original failure repeated the fast cap despite a larger caller budget.
The disconnected control formerly restored topology after proving deadends.
Independent fixtures now preserve those marks without invalidating the positive case.
In-place source leases are checked by the cancellation and ownership controls.
