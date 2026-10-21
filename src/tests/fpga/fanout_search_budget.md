# Fanout source-alternative starvation

`fanout_search_budget.cpp` reduces the Example 1 run-4 failure in which the
first source exit consumed the entire forward search budget before another
free exit was attempted. This is a mechanism regression, not a device-model
import. Every tile, matrix and node name is abstract.

## Fixture

Three fixed logic cells, one signal with two sinks, and a 27 by 15 tile grid:

- `driver` at `(0,1)` first routes its Basic trunk to `trunk_sink` at `(1,1)`.
  The completed trunk's destination has no branching exit.
- `fanout_sink` at `(0,14)` must therefore branch at the source.
- The first free source exit (`node_3_110`) enters a 210-state tree. Its spine
  has one dead-end leaf per position. All of it lies outside the sink's
  Docking radius; all arrivals use distinct tiles. The maximum path length
  is 106 hops, so a 200-hop depth limit would not exhaust this tree early.
- The later free exit (`node_3_120`) goes via `(0,0)` to the sink. This route
  has two inter-tile hops, with no competing owner or required preemption.

The fixture uses actual numeric topology, named intra-tile connections,
resolved jumps, RTL connectivity, and production `routeDesign()`. It checks
the source candidate order rather than assuming an order from node names.
It does not copy the router's search implementation or increase its budget.

## Cases

| CTest | Configuration | Required outcome |
|---|---|---|
| `fpga.fanout_search_budget` | 210-state misleading tree plus valid alternate | Both sinks routed |
| `fpga.fanout_search_budget_control` | Same graph, only the misleading source edge omitted | Both sinks routed through the alternate |
| `fpga.fanout_search_budget_short` | Misleading tree reduced to 20 states | Search backtracks and routes the alternate |
| `fpga.fanout_search_budget_blocked` | Valid source exit removed | Finite failure, no false physical deadend or leaked leases |

The main case is an ordinary success-contract regression: no `WILL_FAIL`,
skip, or accepted-incomplete result. On the unfixed router it verifies the
exact Fanouts failure, 200 accepted search edges, no
timeout/stagnation, no missing concrete connection names, a completed Basic
trunk, and an uncommitted failed suffix before reporting the reproduction.
Every case audits all tile ownership and checks that fixed endpoints stayed
in place. Successful cases must use the alternate source exit.

Run:

```sh
cmake --build build --target grounding_preemption -j2
ctest --test-dir build -R '^fpga.fanout_search_budget' --output-on-failure
```

For the decision trace, run from a separate artifact directory:

```sh
SCALEPNR_DEBUG_ROUTE_NET=fanout_sink \
SCALEPNR_DEBUG_FANOUT_NET=fanout_sink \
/absolute/path/to/build/grounding_preemption search_budget
```

The independent retained-island lease-loss finding from the same Example 1
investigation is not modeled here. The original reproduction changed no
production behavior; the subsequent search fix is described below.

## Observed result before a production fix

- Main regression: **FAIL**, 4.29 seconds, exact 200-state failure reproduced.
  Decision replay records `steps=201, max_steps=200`, deepest failed state
  at depth 105, and exactly one departure from the source (exit 110).
  Exit 120 is never attempted, matching the real failure's search behavior.
- Exit-removal control: **PASS**, 3.92 seconds.
- Short-tree control: **PASS**, 3.88 seconds.
- The five existing trunk-handoff/shared-input-repair regressions also pass.

Summary: `build/search_budget_results.log`. Each test has an isolated working
directory under `build/tests/`; the failing case produces the normal failure
report and image. `build/tests/search_budget/trace.log` contains an additional
decision-level replay with the existing diagnostic flags.

## Fix and verification

Whole-connection forward searches now reuse their speculative state arena on
return to the root. Each starting SRC receives its own bounded 200-state
attempt. When an attempt reaches that quota, its root exit is marked tried
only for the current search; no physical SRC deadend is published. The router
then tries the remaining exits of the same task. Depth remains bounded at 200
and stage cancellation is checked between attempts. Failure coordinates are
stored by value so reclaiming the arena cannot invalidate the diagnostic.

All four cases pass after the fix. The main trace explicitly shows:

```text
starting exit budget exhausted: src=110, states=201, retry_root=true
accept step: from=(0,1), depth=0, ... src=120 ... next=(0,0)
search end: completed=true, final_step=2, steps=3, max_steps=200
```

The five related handoff/shared-input regressions also pass (9/9 total,
17.71 seconds). Fixed-run logs are `build/search_budget_fixed_results.log`
and `build/tests/search_budget/trace_fixed.log`.

The full-suite replay also exposed an avoidable search when every destination
input pin was already leased. Grounding cannot use such a pin, even with
transit preemption. The numeric search now applies Grounding's pin-lease
filter before exploring source alternatives. Shared-terminal attachment and
owner repair remain in their existing earlier handling. The fail-fast
regression checks that a fully leased target generates zero speculative
edges; its five routing modes still fail immediately without processing the
next task. Together with the search-budget, shared-input, immediate-Docking,
source-branching and deadend-commit regressions, all 11 focused tests pass
(20.68 seconds; `build/search_budget_pin_guard_tests.log`).

The final complete CTest run passes **112/112**, including all 11 routing and
placing puzzles, in 2241.14 seconds (`build/search_budget_final_tests.log`).
The 50K-cell routing cases take 240.63 seconds (50% fullness) and 249.97 seconds
(90% fullness); the fanout forest takes 265.36 seconds. Their original
300-second deadlines are unchanged. The Python script regressions pass
**56/56** (`build/search_budget_python_tests.log`).

The subsequent fresh Example 1 replay also completes the formerly failing
connection: a 200-state starting-exit attempt yields to other exits, then
Docking completes the seven-fragment route. Example 1 as a whole still fails
later in run 4 on a different Fanouts connection; this is not a claim of full
design routability. Exact invocation, traces and the new failure are recorded
in `tests/prjxray/1/build/search_budget_fixed.B6OB8rVP/result.md`.
