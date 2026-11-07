# Two-clock placement validation results

All listed designs passed fresh Yosys synthesis, whole-design Outline,
chunked packing/smearing, Sorting and Swapping. Clock periods are fixed
by family in [README.md](README.md); final setup acceptance is WNS ≥ −0.10 ns.
These are generic placement estimates, not routed silicon timing.

| Size | Designs | Mapped cells (min–max) | Tile grids (min–max) | Worst count error | Minimum WNS (ns) | Runs | Sorting moves | Swapping moves |
| --- | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 1k | 11 | 922–1,091 | 7×7–10×10 | 9.10% | 3.429 | 20 | 0 | 0 |
| 5k | 11 | 4,850–5,120 | 16×16–21×21 | 3.00% | 2.966 | 65 | 0 | 0 |
| 10k | 11 | 9,920–10,250 | 23×23–30×30 | 2.50% | 2.722 | 125 | 0 | 0 |
| 30k | 11 | 29,796–30,260 | 39×39–51×51 | 0.87% | 2.264 | 361 | 0 | 0 |
| 50k | 11 | 49,873–50,204 | 50×50–66×66 | 0.41% | 1.778 | 599 | 0 | 0 |
| 80k | 11 | 79,782–80,200 | 63×63–83×83 | 0.27% | 1.169 | 954 | 0 | 0 |
| 100k | 11 | 99,762–100,028 | 70×70–93×93 | 0.24% | 0.561 | 1,183 | 0 | 0 |

A stage may make zero moves when its clock already closes or no legal
improving candidate is accepted; the final combined result must pass.
Accepted swaps leaving an actionable deficit receive a final Sorting pass.
Every test still verifies that each stage executed and that all admitted cells
are legal, fixed cells stayed fixed, and final timing has no unplaced edges.

[validation_2clk.json](validation_2clk.json) contains per-design measurements, stage
intervals, primitive mixes, Yosys version and source/binary hashes.
Intervals include validation and orchestration and may come from concurrent
test runs; they are not isolated performance benchmarks.

Tested placer SHA-256: `45b92f4b1995fea45031e5df680138c96b9489582da66a3ac9e1afbcd2d44956`.

## Per-clock setup checks

| Size | Minimum clk_a WNS (ns) | Minimum clk_b WNS (ns) |
| --- | ---: | ---: |
| 1k | 3.429 | 5.195 |
| 5k | 2.966 | 4.885 |
| 10k | 2.722 | 4.700 |
| 30k | 2.264 | 3.775 |
| 50k | 1.778 | 2.750 |
| 80k | 1.169 | 1.857 |
| 100k | 0.561 | 1.175 |

## Dual-clock FIFO crossing checks

| Size | Direction | Endpoints | Limit (ns) | Worst delay (ns) | Slack (ns) |
| --- | --- | ---: | ---: | ---: | ---: |
| 1k | clk_b → clk_a | 12 | 7.000 | 0.240 | 6.760 |
| 1k | clk_a → clk_b | 48 | 6.000 | 0.805 | 5.195 |
| 5k | clk_b → clk_a | 64 | 7.000 | 0.635 | 6.365 |
| 5k | clk_a → clk_b | 256 | 6.000 | 1.115 | 4.885 |
| 10k | clk_b → clk_a | 128 | 7.000 | 0.858 | 6.142 |
| 10k | clk_a → clk_b | 512 | 6.000 | 1.300 | 4.700 |
| 30k | clk_b → clk_a | 388 | 7.000 | 1.875 | 5.125 |
| 30k | clk_a → clk_b | 1552 | 6.000 | 2.225 | 3.775 |
| 50k | clk_b → clk_a | 640 | 7.000 | 2.245 | 4.755 |
| 50k | clk_a → clk_b | 2560 | 6.000 | 3.250 | 2.750 |
| 80k | clk_b → clk_a | 1028 | 7.000 | 3.613 | 3.387 |
| 80k | clk_a → clk_b | 4112 | 6.000 | 3.915 | 2.085 |
| 100k | clk_b → clk_a | 1280 | 7.000 | 4.628 | 2.372 |
| 100k | clk_a → clk_b | 5120 | 6.000 | 4.825 | 1.175 |

Both capture domains are checked independently after every chunked stage; final
crossing delays are recomputed for every launch domain in each endpoint cone.
