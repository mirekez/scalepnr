# Preserved trunk repair must finish in Fanouts

Run with:

```sh
ctest --test-dir build -R '^fpga.preserved_route_repair$' --output-on-failure
```

The fixture uses only synthetic numeric topology and names. A completed,
preserved source tree has 32 sinks and fixed, completed endpoints. Eight paths
share a transit hop; the other 24 branch before it. The original trunk owns a
private prefix, while its seven dependent siblings have shared prefixes.

Production Grounding preempts the shared transit and queues eight repairs.
The requester occupies the released path. The test passes the queued tasks,
without overriding their classification, to the production Fanouts batch.
The victim must use a free bypass while retaining its downstream routing.

Checks:

- All eight queued tasks are gap repairs runnable during Fanouts, including
  the original private-prefix trunk.
- All 32 connections finish; no repair is left in a deferred queue.
- The 24 unaffected branches retain their geometry and ownership.
- The requester keeps its newly grounded path and no fixed endpoint moves.
- Every Tile passes the independent congestion ownership audit.

Before the fix, the private-prefix task is marked Generic and deferred to
Moving. The regression fails with `private-prefix gap repair was demoted to
Generic/Moving`. In the real second-run failure, three such repairs remained
deferred when the router incorrectly reported completion. Gap classification
must depend on retained routing state, not whether the original prefix was a
shared replica. The scheduler also checks deferred work before claiming success.

## Verification

The regression failed before the fix with the diagnostic above, then passed
through the production Fanouts batch after the fix. The selected routing and
incremental-PnR suite passed all 26 tests.

Example 1 replay confirmed the same cause and repair: run 2 previously reported
1,271 completed task attempts while leaving three Generic-classified repairs
in its deferred Moving queue. After the fix it completed 1,274 attempts with
the same 21 preemptions. Fanouts took 16.420 seconds over six passes and the
run's final connection validation passed. The whole-design replay continued
to a separate Basic-routing failure in run 3; it did not fully route.
