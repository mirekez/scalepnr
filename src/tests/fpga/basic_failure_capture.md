# Capture the actual failed task without predicting its name

Example 1's timed Sorting made a targeted replay accept two moves rather than
three. The previously failing signal then routed and a different later signal
failed. A diagnostic filtered to the previous name could not explain the new
failure. That was not a routing fix or reproduction of the previous snapshot.

`fpga.basic_detour_failure_capture` reuses an abstract captured free detour,
deliberately calls `failRouting()` and enables only the new failure-time
capture. It must record actual endpoints, their ownership and the numeric
frontier/witness without requiring a preselected net. The original failure
must still be thrown; requester bindings and all foreign owners stay intact.

An initial harness omitted `preserve_routes`, so the production failure
handler exited before the assertions. That run was discarded, not counted
as RED. With throwable failure mode enabled, linking the corrected test to
the pre-change production handler reaches the expected missing-capture
assertion. The new handler passes. A separate positive Grounding preview
uses a real transit obstruction and verifies no victim cut or route attachment.

This tests diagnostics and snapshot fidelity, not a new routing algorithm.
All routing acceptance limits remain unchanged. A discovered witness is not
silently committed, and a failed bounded probe is not called disconnection.

Validation: the corrected old-handler test reaches the missing-capture
assertion. Both safety cases pass with the new handler, and all 105 related
routing checks pass. After adding source/sink ownership output and forcing
existing policy-rejection diagnostics only during this failed-task replay,
the final capture, transit-preview and original-choice controls pass again
(three tests, 33.48 seconds). Real snapshot investigation remains required.
