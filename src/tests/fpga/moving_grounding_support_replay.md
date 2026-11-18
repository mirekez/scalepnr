# moving_grounding_support_replay

Moving candidate replay and grounding support.

- Exercise destination Moving with retained and sibling anchors.
- Use both available and occupied packing positions.
- Install real owners for blocked terminal support resources.
- Retain the original input after a nested support failure.
- Replay every failed candidate for diagnostics.
- Check original-position exclusion and no-resource explanations.
- Require reports to include candidate coordinates and packing decisions.
- Blocked resources must identify their actual net and binding.
- Generate valid PNGs for the recorded candidate positions.
- Each anchor has a separate forward trace invocation.
- Sum expanded and visited counters across those invocations.
- The complete log must account for every expected frame.
- No replay trace may be cancelled or leak its diagnostic scope.
- Keep successfully retained routing available for subsequent work.
- Compare live leases and preserved fragments after replay.
- Audit the real resource owners throughout the fixture.

History:

- Revised the existing regression for recursive routing and immediate deadends.

Coverage:

- Uses production entry points and explicit ownership assertions.

Fixture connectivity and node names are vendor-agnostic.
