# Retained island landing lost during sibling replacement

## Recorded failure

Example 1 replay `gap_162946_audit.YXIlsO4x` fails in run 4 on binding 5
of canonical net 162946. The interrupted route has a gap from
`(57,29)/DST3809` to `(50,32)/DST3619` and ends at `(41,32)/LOCAL113`.
Repair exits before source search because the latter boundary has owners
from two different drivers: the interrupted net and canonical net 164336.
The 192/211-node boundary audits reported no missing leases or registrations,
but did not detect incompatible drivers.

The route history identifies the first corruption, not just its final effect:

1. Binding 1 owns the landing `(50,32)/DST3619` on an island after its first
   gap. Binding 5 retains the same arriving hop and a private FORK borrowing
   that DST (`owns_dst=false`).
2. `repairFromSource` replaces binding 1 with a complete new route.
   `promoteSurvivingSourcePrefix` stops at the first non-shared GAP record.
   It therefore never transfers the island's landing to binding 5.
3. During old-route cleanup the DST lease changes from set to clear
   (`HISTORY_BEGIN id=174`), although binding 5 still references it.
4. A later preemption extends binding 5's gap to that DST and makes its
   retained fragment an owner again, without restoring the lost lease.
5. Another driver takes the falsely free node. Binding 5 subsequently fails
   the correct same-source boundary check.

## Regression

`fpga.fanout_island_landing` imports the recorded 13- and 30-fragment routes,
the common source trunk, two partial-cut boundaries, and the real replacement
path including its intermediate LOCAL bridge. All names are generated
abstract `node_<number>` labels. Unrelated routes and congestion are omitted;
this is a reproduction of the ownership transition, not the whole chip.

Production partial cuts create the gaps and landing ownership. A real
registered obstruction forces source replacement. Before the fix it fails:

```text
BUG: source replacement released a surviving fork's DST beyond a gap
```

After repair, the test verifies continuity and every topology arc of the
14-hop replacement, checks the surviving landing and its owner, extends
the cut through that landing, audits the affected Tiles, and tears down the
remaining sibling without damaging the repaired route. Cancellation is also
checked for no mutation.

The fix compares physical fragments across identical GAP records rather
than treating the metadata `shared=false` as the end of the common path.
GAP records own no resources. Existing incoming-DST ownership on a survivor
is preserved when transferring ownership. This extends the existing linear
prefix comparison, without a device-wide scan or new cache.

The later unequal-cut regression (`fanout_cut_island.md`) strengthens partial
cuts to give retained tails independent ownership. To keep this test checking
source replacement rather than relying on that earlier promotion, it restores
the recorded borrowed-DST flags after the cuts while the parent still owns the
DST, then audits that valid state before replacement. The cut regression tests
the new invariant separately; the source-replacement checks are not relaxed.

`fpga.fanout_island_audit` separately injects conflicting driver claims and a
missing borrowed-DST lease to test the read-only diagnostic independently of
the ownership flags. Same-source aliases must remain valid; different output
ports on the same cell are distinct sources.

The stronger audit also exposed an overly broad foreign-owner negative
fixture in `fanout_private_grounding`: it copied the complete source route
under another driver while intending to conflict only at the sink. That
control now uses a separate incoming route and requires exactly one reported
conflicting LOCAL. Positive fixtures require zero conflicts. The negative
case still checks rejection, unchanged state, and protection of the foreign
terminal input; its intended rejection is not relaxed.

## Validation

The rebuilt full suite passed **144/144 tests**, including all eleven puzzles,
in 2081.16 seconds, with unchanged limits and thresholds. The separate added
14-hop continuity assertions also passed. Logs: `/tmp/island-all.log` and
`/tmp/island-all.xml`. Example 1 was restarted after that successful gate;
this validation alone is not an end-to-end Example 1 success claim.

The real replay `island_landing_fixed.q13ALaqG` completes the formerly failing
binding 5 as a 20-fragment route. Run 4 finishes in 166.906 seconds of routing,
bringing the completed design to 3,699 cells. Run 5 subsequently fails on a
different clock-enable fanout, with no timeout/stagnation and no ownership
errors in its reported sink-area Tiles. That new failure is being investigated
separately; the complete example is not yet successful.
