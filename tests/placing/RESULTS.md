# Placement validation results

All listed designs passed fresh Yosys synthesis, whole-design Outline,
chunked packing/smearing, Sorting and Swapping. Clock periods are fixed
by family in [README.md](README.md); final setup acceptance is WNS ≥ −0.10 ns.
These are generic placement estimates, not routed silicon timing.

| Size | Designs | Mapped cells (min–max) | Worst count error | Minimum WNS (ns) | Runs | Sorting moves | Swapping moves |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1k | 10 | 931–1,097 | 9.70% | 2.962 | 17 | 0 | 0 |
| 5k | 10 | 4,865–5,120 | 2.70% | 2.550 | 60 | 0 | 0 |
| 10k | 10 | 9,718–10,177 | 2.82% | 1.801 | 116 | 0 | 0 |
| 30k | 10 | 29,952–30,228 | 0.76% | 0.668 | 328 | 0 | 0 |
| 50k | 10 | 49,920–50,215 | 0.43% | -0.032 | 547 | 0 | 0 |
| 80k | 10 | 79,939–80,089 | 0.11% | -0.097 | 868 | 240 | 24 |
| 100k | 10 | 99,840–100,019 | 0.16% | -0.097 | 1,083 | 314 | 34 |

A stage may make zero moves when its clock already closes or no legal
improving candidate is accepted; the final combined result must pass.
Accepted swaps leaving an actionable deficit receive a final Sorting pass.
Every test still verifies that each stage executed and that all admitted cells
are legal, fixed cells stayed fixed, and final timing has no unplaced edges.

[validation.json](validation.json) contains per-design measurements, stage
intervals, primitive mixes, Yosys version and source/binary hashes.
Intervals include validation and orchestration and may come from concurrent
test runs; they are not isolated performance benchmarks.

Tested placer SHA-256: `60064bdb291b56e8bb2001d0e43916cf999349f2e6eeab5854adc6c61e5d2a27`.
