# destination_forward_child_retry

Resume the parent cursor after one physically dead child.

- Build a root with two numeric SRC alternatives.
- Aim directly toward the first child so angle priority chooses it.
- The first child ends at a JOINT with no outgoing connection.
- It has no usable terminal LOCAL hidden by placement policy.
- The second child reaches the requested target through another jump.
- Accept only the requested target in the docking callback.
- Record the forward trace decisions.
- Check that the first SRC is tried before the second.
- Check successful completion after returning from the dead child.
- Count frames including the explicit JOINT leaf.
- Check that one exhausted SRC is persisted.
- Check the selected-edge count.
- Check the number of returned route fragments.
- The first returned jump must use the live root exit.
- The second returned jump must reach the target.
- The returned path must not include the discarded dead suffix.
- Preserve the root DST lease.
- Compare every other SRC, DST, JOINT and LOCAL mask.
- No pin lease may remain after probing.
- The test checks recursion and cursor resumption, not shortest paths.

History:

- Made the rejected child physically dead instead of a declined free terminal.

Fixture connectivity and node names are vendor-agnostic.
