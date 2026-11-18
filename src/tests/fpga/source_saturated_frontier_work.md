# source_saturated_frontier_work

Recursive source label ceiling.

- Exercise source completion with sixty-four, 128, and 256 labels.
- Numeric LOCAL-to-JOINT-to-SRC alternatives exercise recursive choices.
- Two JOINT approaches share a SRC with a foreign-owned DST landing.
- The disconnected mode must consume its bounded allowance.
- No ancestor may continue scanning after the cap is reached.
- Immediate cancellation must expand no node and change no state.
- A direct LOCAL sink should finish after two expanded nodes.
- Do not require successful direct routing to consume its whole budget.
- A retained-tail control reaches its join at the last permitted label.
- Its count includes two LOCALs, repeated SRC attempts, and one initial DST.
- The retained continuation must still be checked at that join.
- A foreign-owned terminal approach must not be traversed.
- Failure must preserve route fragments and registrations.
- Both persistent deadend masks remain unchanged during repair.
- Use expansion and edge counters instead of wall-clock limits.
- Audit ownership after every mode and budget.
- A zero-hop cross-Tile request must stop at its root without edges or deadends.
History:

- Updated the existing case for recursive routing.
- Require five expanded nodes for the two JOINT approaches, without a new mark.

Implementation:

- Exercise the production routing entry points directly.
A label cap before that SRC is attempted must leave its deadend bit clear.
