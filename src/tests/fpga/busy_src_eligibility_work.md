# busy_src_eligibility_work

Busy SRC ownership query work.

- Use eight, sixteen, and thirty-two distinct incoming DSTs.
- Populate each hub with thirty-two, sixty-four, or 128 busy SRCs.
- Every busy resource belongs to an installed physical route.
- Also register unrelated foreign transit bindings in that Tile.
- Check a free exit, closed exits, shared routing, and net aliases.
- The free exit can finish at the first incoming DST.
- A shared SRC is reusable only from its actual incoming arc.
- The matching shared arrival is deliberately last.
- Read the existing angle order to count checks before success.
- Do not assume equal-angle SRCs have numeric order.
- Count owner visits independently of wall-clock time.
- Increasing arrivals and exits must not multiply whole-owner scans.
- Keep all foreign routes and sibling fragments unchanged.
- Check the exact successful physical hop count.
- Immediate cancellation must leave ownership unchanged.
- Audit all occupied resources and route registrations.
- Derive both the shared arrival and exit positions from loaded angle priority.
History:

- Revised the existing regression for recursive routing and immediate deadends.

Coverage:

- Uses production entry points and explicit ownership assertions.

Fixture connectivity and node names are vendor-agnostic.
