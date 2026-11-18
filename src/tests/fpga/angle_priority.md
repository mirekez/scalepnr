# Angle priority

Exercise the shared Crossbar angle ordering and next-hop cursor.
The requested direction determines priority before numeric node identity.
Shorter jumps precede longer jumps with the same angle.
A typed entry can select only SRCs reachable from that entry.
An unrelated LOCAL's better-directed SRC must remain excluded.

## Cursor contract

Each call consumes one SRC from a typed connectivity and availability mask.
Exhaustion returns no SRC and never repeats an earlier choice.
Opposite-direction choices remain available after preferred choices.
Resolved geometry takes precedence over the SRC's encoded index.
Long resolved jumps must retain their physical direction when bucketed.
Bucket coordinates cover the symmetric range from -7 through +7.
A displacement component of -8 must be normalized into that range.
Check horizontal, vertical, diagonal and mixed boundary displacements.
Include positive boundary controls to preserve symmetric behavior.

## Ordering coverage

Dense masks retain every lane and every eligible SRC exactly once.
Node-specific order must agree with the filtered complete order.
DST, LOCAL and JOINT masks filter occupied and deadend SRCs identically.
Live mask changes remove untried choices without repeating returned SRCs.

## History

History: added typed-mask controls for occupancy, deadends and angle fallback.
