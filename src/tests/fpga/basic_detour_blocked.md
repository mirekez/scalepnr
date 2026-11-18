# basic_detour_blocked

Route the recorded detour through an internal LOCAL input bridge.
Keep the original numeric frontier and resolved SRC geometry.
Use abstract node names, fixed endpoints and registered foreign owners.
Missing captured exits terminate at real owned obstructions.
Independent reachability checks whether the sink is connected.

## Search contract

The caller requests at most nineteen physical Tile transitions.
The accepted route must satisfy that same nineteen-hop bound.
The normal positive request allows 4194304 recursive labels.
The larger-budget positive control allows 8388608 recursive labels.
The disconnected negative control retains its 32768-label cap.
Angle priority determines the order; these bounds do not reorder choices.
The blocked variant disconnects the final input continuation.

## State and ownership

Immediate cancellation must not install a route or alter foreign owners.
Success must install exactly one complete source-to-input binding.
The required internal LOCAL bridge must remain in the accepted path.
The source and sink must retain their fixed placements.
Audit all installed leases and registered route ownership.
Unrouting must release the requester without damaging other owners.

## History

Aligned the caller hop bound with the existing nineteen-hop assertion.
