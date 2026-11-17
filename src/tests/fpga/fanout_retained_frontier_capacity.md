# fanout_retained_frontier_capacity

Purpose: distinguish a bounded repair failure from physical disconnection.
The captured frontier has a valid completion beyond a restricted search budget.
Implementation: `fanout_retained_frontier_regression(true, false)`.
Run: `ctest --test-dir build -R '^fpga.fanout_retained_frontier_capacity$' --output-on-failure`.

The fixture loads abstract numeric connectivity from the retained frontier capture.
It imports 105,671 recorded arcs and a thirty-nine-fragment retained binding.
The source is (82,162)/LOCAL142; the sink is (58,89)/LOCAL84.
Imported fragments reconstruct the same numeric ownership and endpoint pins.
The production repair receives a thirty-thousand-label budget.
The normal production budget and routing choice order are unchanged.
Detailed source diagnostics report whether the search limit was reached.

If bounded repair fails, every retained route and lease must remain unchanged.
Deadends proved before the cap may remain marked immediately.
No previously recorded deadend bit may be removed.
Independent read-only reachability must establish a physical sink witness.
It receives forty thousand states; its printed witness must use at most two hundred hops.
Its state snapshot is taken after the failed production attempt.
The diagnostic must leave that entire snapshot unchanged, including deadends.
If bounded repair succeeds, the route must be physically complete.
A successful route must preserve both original endpoint coordinates.
Unrouting that completed binding must release its owned resources.
Owner and registration audits run around every state comparison.
The sole binding may re-enter its own detached island through a different input arc.
History: the case records a known bounded-search failure with a reachable sink.
An old equality check incorrectly rejected newly proved deadends after failure.
The route-state check permits only added deadends; the diagnostic stays read-only.
