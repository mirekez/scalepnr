# A soft failure must end the routing stage

A whole-connection search can return false after calling the soft diagnostic
handler. Previously, Basic and Fanouts continued with the next connection and
retried the failed connection in later passes, potentially until the stage
budget expired. A failure report therefore did not identify a stage boundary.

A soft failure now ends the current stage immediately. Its unfinished
connection stays first and enters the corresponding Moving recovery stage.
Recovery must finish that connection or fail routing. Hard failures stop
immediately. An internal rejected search candidate remains an ordinary part
of the search, rather than a whole-connection failure.

## Regression checks

- `fpga.routing_soft_failure_handoff` submits a blocked input followed by a
  routable connection to both Generic and Fanout batches. Each must attempt
  exactly one connection, report failed-stage status, retain that connection
  first, and leave the later connection untouched. The test checks unchanged
  live resource leases, restored task scope, failure text and a nonempty PNG.
  A separate direct call proves the later connection really is routable.
- `fpga.routing_fail_fast` also checks terminal failure in Source Moving,
  Destination Moving, Const and nested Generic/Fanout recovery batches.
- `fpga.routing_soft_failure_scheduler_heartbeat` and
  `fpga.routing_soft_failure_scheduler_batch` run the real stage scheduler.
  A complete trunk serves an unreachable suffix and a later suffix with a
  proven free direct path. The unreachable suffix must reach fatal Moving
  recovery before the later suffix is routed, without waiting for a deadline
  or watchdog. Ownership is audited before and after the failed run.

All fixture node names are synthetic. The batch regression was run against
production behavior before the fix and failed because the second connection
was attempted after the first failed.
