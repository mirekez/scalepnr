# forward_observer_work

Check placement diagnostics during a long internal LOCAL path.

- Begin at one source-connected LOCAL anchor.
- Connect 1,024 LOCAL nodes inside one crossbar.
- Put one physical SRC exit at the end of the chain.
- Land on one DST in the next Tile.
- Give that DST one possible placement terminal.
- Run once with the terminal accepted.
- Run again with the terminal declined.
- Invoke the observer immediately before each placement probe.
- Require one observer call for the one actual probe.
- Require the observer to precede its probe.
- Keep diagnostic copies limited to occupied path Tiles.
- The LOCAL chain must not multiply snapshot copies.
- Count expanded physical hops rather than internal LOCAL nodes.
- Accept only the route whose terminal probe approves placement.
- Preserve the root and internal connectivity during failure.
- Release every temporary lease after either attempt.
- The long chain requires Docking's internal full search.
- The forward angle trace remains one frame per physical hop.

History:

- Added a focused observer workload for long LOCAL connectivity.
- Restored the observer call before terminal probes.
- Counted two physical frames after the Docking internal search was added.

The test uses no vendor-specific fabric names.
