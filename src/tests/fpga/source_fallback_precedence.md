# Preserve an original successful terminal choice

The blanket half-bias experiment changed a real run-4 numeric completion from
23 fragments / one preemption to 27 fragments / two preemptions. Run 4 still
finished, but later routing-aware placement changed, run-5 routing took longer,
and the first run-6 failure moved to a different connection. Clean ownership
audits do not make such choice changes an improvement.

This small 14-by-6 abstract model isolates that decision-ordering problem,
not the different connection's complete physical snapshot. It has an existing
source trunk and two feasible terminal approaches. The geometrically closer
approach uses more free internal edges but needs only one transit victim.
The cheaper, farther approach needs two independent transit victims. Every
busy node has a real complete route, with independent ownership audits.

Original priority reaches the one-victim choice first. Blanket weaker
geometry reaches the two-victim choice first: the test fails with
`replaced a previously successful one-victim approach with a two-victim choice`.
This is the valid RED result, not a fixture setup rejection.

Keeping the original attempts first and using alternate priority only after
their failure passes. It also preserves the source trunk and both unrelated
completed routes, repairs the displaced transit through its private outlet,
and audits requester teardown. All node names are abstract. No persistent
planner, scheduling mechanism or extra candidate cache is involved.

The final original-first implementation passed all 191 tests, including all
eleven puzzles, with unchanged limits. A fresh real Example 1 run remains a
separate required validation; the previous blanket trial was not successful.
