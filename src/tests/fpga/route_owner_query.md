# Short-circuit routing ownership queries

Run with:

```sh
ctest --test-dir build -R '^fpga.route_owner_query$' --output-on-failure
```

The synthetic 32-sink fixture in `partial_preemption.cpp` compares the fast
predicate query against exhaustive physical-owner enumeration for every leased
LOCAL, DST, SRC and JOINT. It checks both authoritative binding registrations
and the legacy net-registration path. All signal and resource names are abstract.

Checks include:

- Empty/orphan nodes reject even when the predicate would accept every owner.
- A rejection stops after the first disqualifying owner; acceptance visits all
  owners. A later foreign source cannot be hidden by an acceptable first owner.
- Shared replicas do not own transit resources, while an explicitly retained
  shared landing still owns its DST.
- Explicit numeric edges own their output, not their input reference.
- A matching numeric ID on another Tile does not satisfy the query.
- Null/stale registrations are ignored, and Takeoff is not treated as transit.
- The fixture's final congestion audit is clean after all temporary controls
  have been restored.

The production optimization rejects unrelated numeric fragment IDs before
coordinate checks, and lets search eligibility reject on the first ineligible
physical owner. It adds no persistent ownership cache. Exhaustive enumeration
remains available for diagnostics and release/commit operations. Opt-in watched
node history remains exhaustive even when eligibility uses the fast query.

The test also prints elapsed time for 20,000 rejected queries, without asserting
a machine-dependent speed threshold. An initial Debug measurement was 142.6 ms
for exhaustive enumeration and 5.8 ms for the short-circuit query. This is a
controlled first-owner-rejection case, not a claim about full-router speed.

On 2026-10-09 all 15 rebuilt focused tests passed: general routing, landing and
discard ownership, ownership history, Grounding/preemption, partial-route
repair and its gap/suffix cases, the new query, and Docking.

## Example 1 measurement

The optimized replay used the same synthesized design, 2,000-cell run target,
600-second stage deadline, Debug build and 50 Hz sampler as the baseline
profile. The first five Fanouts pass-end task counts were identical:
1,970, 1,555, 1,408, 1,214, 894.

| Checkpoint | Before | After |
| --- | ---: | ---: |
| First three Fanouts passes | 69.74 s | 52.72 s |
| First four Fanouts passes | 124.54 s | 91.61 s |
| Fifth pass alone | 152.86 s | 83.37 s |
| First five Fanouts passes | 277.40 s | 174.98 s |
| Last sixth-pass completed-task heartbeat | 164 | 369 |

Both runs still reached the 600-second Fanouts deadline in the first PnR run;
this is a measured performance improvement, not full-routing success. Heartbeat
counts are lower bounds, not the exact number completed before cancellation.
The optimized profile still spends about 52% of Fanouts CPU in ownership
predicates and 16% resolving jump targets. These whole-budget percentages cover
more repair work than the baseline, so the equal-work pass timings above are
the appropriate speed comparison.

Baseline artifacts: `tests/prjxray/1/build/fanouts_profile.d83Jpary/`.
Optimized artifacts: `tests/prjxray/1/build/owner_opt.CgUF9aCm/`.
Both contain stage markers, sampling reports and routing logs. The ownership
query also passed with watched-JOINT history enabled, preserving owner proofs.
