# gap_source_completion_policy
Purpose: check source completion of a retained GAP and its cut policy.
Use two straight numeric corridors of lengths seventy-two and ninety-six.
Each requester starts with a retained source-connected prefix.
Its missing continuation ends at a mapped sink LOCAL.
A foreign completed route occupies the required terminal DST.
A second terminal entry exists but is unreachable from this source.
This prevents confusing terminal availability with source reachability.
Run each corridor with Grounding cuts allowed and refused.
An immediately cancelled call must leave all routing unchanged.
Call the original stateful cancellation callback exactly once.
Do not copy its mutable callback target while profiling.
The normal attempt must perform one free source search.
That search must fail at the occupied terminal approach.
The source attempt must not depend on a cached frontier.
A refused cut must leave every binding and lease unchanged.
Persistent proved deadends may remain after a failed attempt.
An allowed cut must identify the real terminal owner.
Previewing a cut must not apply it.
Successful completion must apply exactly one cut.
The requester must finish as a complete driver-to-sink route.
Its path must follow the full numeric corridor.
The retained prefix must stay connected to the source.
Every live occupied node must retain a real route owner.
Audit both the requester and the displaced binding.
The trace must report one source search for each normal repair.
It must report no reused search frontier.
Reuse this fixture for repair profiling and filter checks.
History: replaced obsolete frontier-cache expectations with completion checks.
History: retained cancellation, cut, path, and ownership assertions.
