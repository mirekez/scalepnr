# Vendor-independent placement development plan

Implement and validate sizes in order: **1k → 5k → 10k → 30k → 50k → 80k → 100k**.
Do not generate the next size until the current size passes synthesis and placement.

1. Study Estimate, whole-design Outline, pre-smearing/exact packing, Sorting,
   Swapping and incremental Pnr; establish a generic Yosys synthesis fixture.
2. For each size, calibrate ten standalone SystemVerilog designs against actual
   post-synthesis cells (±10%, excluding artificial test I/O and constant sources).
3. Run each design through whole-design Outline once and the existing incremental
   planner and placement stages. Check finite/in-bounds outline coordinates,
   complete legal resource assignment, fixed-cell preservation, full timing
   endpoint coverage, no final unplaced timing edges and the declared setup limit.
4. Investigate failures at the first failing stage; fix implementation defects,
   retain focused regression checks, and document evidence in PROBLEMS.md.
5. Record measured counts and stage results. Re-run earlier sizes when shared
   implementation changes affect them. Run relevant existing placement/PnR tests.

The fixture uses an abstract FPGA resource and delay model, not a claim about
silicon Fmax. Clock targets are declared per family and held across sizes.
Memory and arithmetic are synthesized to portable soft logic; hard BRAM/DSP
packing needs a separate architecture model. No vendor primitives or keep-based
padding are used in the RTL.

Progress (measured, not just generated):

- 1k: ten designs pass, including the corrected FIR pipeline.
- 5k: ten designs pass.
- 10k: ten designs pass.
- 30k: ten designs pass after shared-logic, deep-traversal and incremental timing
  fixes. Earlier tiers rerun with the same placer binary.
- 50k: ten designs pass after the shared-cone packing fix.
- 80k: ten designs pass after capture grouping and tied-critical-path repairs.
  The final stencil has 80,025 cells, 82 runs and WNS −0.097 ns.
- 100k: all ten designs pass with the final binary; measured counts are
  99,840–100,019 cells. The stencil exposed a late stage-composition issue.
  Its complete CTest now passes all 102 runs at WNS −0.097429 ns after one
  conditional post-swap Sorting pass.

Final validation is complete: all 70 designs were freshly synthesized and
placed with the same final binary, covering 3,019 placement runs. All 71 suite
CTests (including Yosys setup) pass in 2,777.53 seconds with `-j12`, plus all
29 focused regressions. Source and binary hashes were verified when recording
the report. No Sorting or Swapping stage regressed WNS in the measured suite.

See `validation.json` for exact counts, model periods, stage movement counts,
WNS/TNS, source hashes and the tested placer binary hash. `PROBLEMS.md` records
the defects and the focused regression evidence.
