# moving_fixed_seed_promotion

Purpose: finish a fixed sink after a Fanout task loses its source tree.
The task is promoted to Generic routing without changing either cell.
The fixture has one free source hop and one occupied transit exit.
A separate route owns that transit exit and may be cut by preemption.
The sink is a fixed resource input LOCAL behind two CB jumps.

The first free trace must inspect the reachable sink neighborhood.
Failure in that trace does not prove the occupied exit is permanent.
A persistent SRC deadend must not hide a later eligible cut.
The retry must use the same numeric connectivity as the free trace.
Docking may search within its bounded Grounding square.
A valid bridge cut must keep the foreign takeoff protected.
The displaced route remains repair work after the cut.

An alternate free route is the positive no-cut control.
A protected foreign route is the negative cut control.
Disabled preemption is another negative control.
The promoted task must no longer be classified as Fanout.
Promotion must not move the source or sink placement.
The old sink position remains valid for the resulting route.
The same controls also cover a movable sink at its current position.

This test separates present occupancy from persistent deadend proof.
It exercises complete-route validation after the bridge is selected.
It checks that temporary placement focus is restored.
It checks route ownership after the accepted completion.

History: documented the deadend and preemption retry boundary.
