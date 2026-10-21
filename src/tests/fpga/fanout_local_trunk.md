# Fanout from a local-only trunk

`fpga.fanout_local_trunk` reproduces the Example 1 run-4 failure after the
long Grounding repair fix. The task had one complete seed but zero branching
candidates. The seed consisted of exactly three fragments:

```
(63,69) source pin LOCAL144
(63,69) source-position crossbar fragment LOCAL144, jump=-1
(63,69) sink pin LOCAL127
```

The remote sink was at CB `(58,61)/LOCAL107`. The recorded Fanouts task made
zero search attempts. The collector rejected the source fragment because it
had no existing SRC jump. That condition is appropriate for collecting
downstream DST arrivals, but not for collecting the actual driver LOCAL.

The test installs the three-fragment seed as a registered, audited snapshot,
then calls the production complete-task Fanouts entry point. Geometry is
translated by `(-55,-60)`. Cells and node names are synthetic; unrelated chip
congestion and aliases are omitted. It is not a test of how the local trunk
was initially constructed. A two-hop path from its source to the remote sink
proves that branch discovery, not occupancy, prevents completion.

Before the fix, the test fails with:
`BUG: complete local-only trunk provides no Fanouts source branch`.

The fix removes only the source fragment's `jump>=0` requirement. It retains
the source-position/local checks, complete-seed filtering, foreign-source
ownership guard, deduplication, and downstream-first branching order.
The source route and its local sink must remain complete and owner-clean.
`fpga.fanout_local_trunk_blocked` removes the second hop and requires failure
without leaving partial leases or damaging the seed.

Validation: the unfixed positive case reproduced the missing-branch failure;
both cases passed after the source guard change. The complete 139-test suite,
including all eleven puzzles, then passed without changing limits or timing
requirements.
