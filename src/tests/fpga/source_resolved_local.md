# source_resolved_local

Physical LOCAL transitions in source routing.

- Use the abstract captured 119-node physical connection.
- Its continuation includes many cross-Tile LOCAL links.
- Also retain zero-displacement and differently typed LOCAL targets.
- The resource Tile and routing Tile can be distinct identities.
- The ordinary terminal approach has a real control-route owner.
- A read-only witness must verify the expected physical path.
- Build a separate control with physical LOCAL links removed.
- That graph must be disconnected and publish no route.
- A dead SRC alternative must not hide a physical LOCAL continuation.
- Overlapping reverse-index builds must not duplicate predecessors.
- A narrow view of the complete index must retain the same links.
- Immediate cancellation must leave ownership intact.
- A hop bound below the witness length must reject completion.
- A sufficient explicit budget must complete the numeric path.
- The control-input tree must remain unchanged.
- Audit every live node after both failure and completion.

History:

- Updated the existing case for recursive routing.
- Cover a LOCAL bridge that also offers an unusable ordinary SRC.

Implementation:

- Exercise the production routing entry points directly.
Fixture connectivity and node names are vendor-agnostic.
