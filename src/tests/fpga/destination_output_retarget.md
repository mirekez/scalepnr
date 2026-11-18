# destination_output_retarget

Preserve a relocated driver's usable output when routing its loads.

- Load the captured numeric frontier with 3,527 physical arcs.
- Keep resource Tiles distinct from their routing CB Tiles.
- Install a LUT with six inputs and two output loads.
- Keep the nearby load and distant load at their original positions.
- Preserve the output MUX and LUT packing restrictions.
- Move the LUT through the production Moving Sources entry point.
- Require the captured legal destination position.
- Verify the direct LOCAL162 output has an independent route witness.
- Verify the replacement MUX output remains a closed component.
- Route the constrained distant load as the Generic trunk.
- LOCAL154 is tried before the usable output alternative.
- Its first branch reaches an exitless Tile at coordinate (49,3).
- Assert that Tile has neither SRC jump mappings nor resolved LOCAL links.
- Its internal LOCAL/JOINT paths cannot reach the distant fixed sink.
- The fixed-sink trace must return and continue its source alternatives.
- Do not infer a persistent deadend for another connection's terminal.
- Require the distant load and nearby fanout to complete.
- Require the output triangle to remain complete.
- Preserve the relocated driver's identity in every binding.
- Audit every route's registrations, leases and ownership.

History:

- Added an explicit exitless-Tile assertion while diagnosing recursive exhaustion.

Fixture connectivity and node names are vendor-agnostic.
