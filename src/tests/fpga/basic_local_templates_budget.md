# basic_local_templates_budget

Resolved connectivity for captured LOCAL templates.

- Load the abstract captured free graph and its independent sink witness.
- Keep the original source and sink coordinates.
- The companion data lists LOCAL templates with unmatched neighbor types.
- Such templates do not provide real physical continuations.
- Check physical reachability independently before production routing.
- Immediate cancellation must leave routes and leases unchanged.
- Normal and larger-budget controls use explicit recursive label bounds.
- Use 2097152 labels normally and 4194304 in the larger-budget case.
- The control without unresolved templates keeps the traversable graph.
- The disconnected control removes the sole final sink connection.
- Positive cases must install one complete physical binding.
- Angle priority does not promise the shortest witness route.
- Count hops only when a physical edge changes Tile coordinates.
- Enforce the requested allowance of two hundred Tile transitions.
- Keep endpoint placements fixed and audit all resource ownership.
- Unrouting must release the successful binding without orphan leases.

History:

- Updated the existing case for recursive tracing and persistent deadends.

Coverage:

- Preserves positive completion, negative controls, and ownership assertions.

Each variant uses a separate directory for its search diagnostics.
