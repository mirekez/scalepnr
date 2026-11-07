# Two-clock placement extension

Work through 1k → 5k → 10k → 30k → 50k → 80k → 100k. Generate the next
tier only after every current-tier CTest passes. Revisit prior tiers when a
shared implementation changes.

1. Retain the ten existing families, dividing their useful sections between
   two independent clock domains. Export both results without implicit CDCs.
2. Add an 8-entry, 12-bit dual-clock FIFO family with registered Gray pointers,
   two-stage synchronizers, full/empty control and pipelined bank reduction.
3. Keep each family's original period on `clk_a`; use 1.5× that period on
   `clk_b`. Require actual endpoints in both domains and audit their coverage
   and timing independently. Cell targets apply to the combined design.
4. Add directional maximum data-path delay constraints. For the FIFO, check
   `clk_a` → `clk_b` against 6 ns and the reverse direction against 7 ns.
   Independently recompute every launch-domain contribution, not just the
   selected critical branch. Never replace these constraints with false paths.
5. Validate focused Tcl/timing regressions and the FIFO's data order, wrap,
   full/empty behavior using two clocks with different periods and phases.
6. Run whole-design Outline once, then existing chunked placement and all
   existing legality/frozen-cell checks. Record source, constraint and binary
   hashes with per-clock and per-crossing timing results.

Progress:

- 1k: all eleven two-clock cases pass, plus the FIFO functional check and all
  ten existing single-clock cases.
- 5k: all eleven two-clock cases pass.
- 10k: all eleven two-clock cases pass.
- 30k: all eleven two-clock cases pass.
- 50k: all eleven two-clock cases pass.
- 80k: all eleven two-clock cases pass.
- 100k: all eleven two-clock cases pass; counts range from 99,762 to 100,028
  cells. This tier was generated only after the 80k acceptance gate.
- All 16 focused multi-clock cases pass, including explicit-delay timing and
  Sorting/Swapping repairs. All 44 selected placement/PnR/multi-clock regressions
  pass; the rebuilt main executable also passes its Tcl entry-point check.

Complete: all 77 designs pass fresh synthesis and placement with one verified
placer binary, covering 3,307 placement runs. Every final per-clock slack is
positive (minimum 0.561 ns); every FIFO crossing limit passes (minimum margin
1.175 ns). No Sorting or Swapping stage regresses WNS. The FIFO functional
check and 45 focused regressions, including the main Tcl check, pass as well.
Source, JSON constraint, Tcl constraint and binary hashes are recorded in
`validation_2clk.json`; `RESULTS_2CLK.md` includes per-clock and crossing results.
