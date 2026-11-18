# source_grounding_budget_escalation

Purpose: retry a bounded source search before attempting a Grounding cut.
Implementation: source_grounding_budget_escalation_regression.
Run the fpga.source_grounding_budget_escalation CTest case.

The source retains a completed sibling trunk.
The requested sink lies behind a foreign transit DST.
A free detour initially moves away from the sink.
That detour reaches the final Grounding approach.
A competing corridor has thirteen chained JOINTs.
Eighteen sibling JOINTs then approach one unusable SRC.
All connectivity is numeric and vendor independent.
Every occupied node has an installed route owner.

The four- and eight-label probes cannot reach the useful detour.
The explicit retry has a sixty-four-label budget.
Both ordinary and reused search entry points are exercised.
Both permitted and refused terminal cuts are exercised.
Immediate cancellation changes neither routing nor marks.
The small failed probe leaves the requester uninstalled.
Repair may read deadends but must not write them.
The sufficient retry reaches Grounding through free resources.
Only the accepted complete replacement may cut the terminal obstruction.
A refused cut leaves the original owners intact.
A successful cut installs a complete four-hop connection.
The sibling trunk remains complete and correctly owned.

History: added to expose skipped free-search budget escalation.
History: reduced sibling count after repair stopped creating deadends.
