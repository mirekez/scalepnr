# moving_source_probe

Source completion before unrelated GAP work.

- Retain a source prefix, a disconnected GAP, and its downstream island.
- Provide a separate seven-hop numeric route toward the same sink.
- That route includes an internal LOCAL bridge.
- Its final approach crosses genuine foreign transit.
- Give the unrelated GAP many unusable outgoing alternatives.
- Call source completion directly with its explicit label budget.
- Immediate cancellation must preserve all owners.
- A one-label budget must not complete the connection.
- A six-hop limit must reject the seven-hop witness.
- Build missing-bridge and connected controls independently.
- Do not mutate topology underneath persistent deadend marks.
- Refusing transit preemption must preserve the foreign route.
- Allowing it must complete only when the LOCAL bridge exists.
- Exactly one accepted cut must accompany successful completion.
- The diagnostic trace must contain no exact-GAP expansion.
- Audit ownership; count labels rather than cancellation callback polls.

History:

- Updated the existing case for recursive routing.
- Preserve immediate deadends independently of route leases.

Implementation:

- Exercise the production routing entry points directly.
Fixture connectivity and node names are vendor-agnostic.
