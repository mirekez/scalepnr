# basic_remote_local

Complete the recorded free route through a remote LOCAL network.
Load the captured numeric free frontier and its recorded witness arcs.
The source is (45,13)/LOCAL141; the sink is (37,36)/LOCAL83.
Preserve resolved cross-Tile LOCAL links as physical connectivity.
Do not treat a terminal input LOCAL as a fabric jump.

## Required checks

Independent reachability establishes whether the input is connected.
The blocked variant removes LOCAL77 to LOCAL83 in the sink Tile.
Immediate cancellation must leave bindings and route storage empty.
A successful search must install exactly one complete physical binding.
The final LOCAL77 to LOCAL83 bridge must appear in that route.
The accepted route must satisfy the fixture's 62-hop witness bound.
Neither endpoint may move to obtain completion.
A failed search must leave no partial installed route.
Audit leases, registrations and ownership against the real route.
Unrouting must release the completed binding without stale ownership.

## Diagnostic isolation

Each variant writes its search log in its own working directory.
Parallel CTest cases must not truncate or mix one another's logs.
Diagnostics do not supply connectivity or select an alternate route.

## History

Separated variant working directories to preserve independent traces.
