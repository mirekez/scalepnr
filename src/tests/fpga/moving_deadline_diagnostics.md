# moving_deadline_diagnostics

Deadline diagnostics preserve routing semantics.

- Keep ordinary routing outside the final diagnostic window quiet.
- Enter the last hundred milliseconds with a valid active task.
- Check that verbose diagnostics activate without postponing the deadline.
- The task identity must appear in the trace.
- An expired deadline must still cancel the search.
- Starting another stage must clear diagnostic verbosity.
- Exercise retained destination repair with a genuine transit victim.
- Capture route decisions, physical fragments, and occupancy masks.
- Require real owner proofs for the blocked resources.
- The trace must include both pre-attempt and post-attempt state.
- Verbose output may consume the remaining short stage allowance.
- Such an expiration must remain a failed attempt.
- After ending that stage, resume the same task with its retained state.
- Require eventual complete repair and correct victim scheduling.
- Preserve the driver and destination placements.
- Diagnostic scopes must be released after each attempt.

History:

- Revised the existing regression for recursive routing and immediate deadends.

Coverage:

- Uses production entry points and explicit ownership assertions.

Fixture connectivity and node names are vendor-agnostic.
