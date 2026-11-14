Tests in this folder are very long tests, not regression tests to be run after each change. Find regression tests in src/tests/

Run these workloads explicitly for integration, performance, or acceptance
testing. Do not automatically include the whole folder in the per-change
regression gate.

## Placement Acceptance

| Location | Description |
| --- | --- |
| `placing/1k`, `5k`, `10k`, `30k`, `50k`, `80k`, `100k` | Synthesized placement workloads covering arithmetic pipelines, FIRs, map/reduce, memory lookup, FIFOs, crossbars, CRC, meshes, arbiters, and compare/exchange logic. Check cell counts, packing, timing, and placement-stage performance. |
| `placing/<size>/2clk` | Two-clock variants plus an asynchronous FIFO, with per-clock and cross-domain timing checks. |
| `placing/run.py`, `report.py` | Run selected placement cases and summarize their metrics. Generators and manifests describe reproducible workloads. |

See [placing/README.md](placing/README.md) for prerequisites, suite selection,
acceptance criteria, and result locations. Run bulk placement suites only when
explicitly requested.

## Device Routing And Export

| Entry Point | Description |
| --- | --- |
| `prjxray/test.sh`, `test.tcl` | Synthesize the example design, run scalepnr placement/routing, save its database, and export FASM; produce a bitstream when the conversion tools are available. |
| `prjxray/.run_50k_test.sh` | Rerun the retained 51,749-cell design with chunked PnR, routing-stage deadlines, resource guards, and failure diagnostics. Requires the saved input design or explicit input overrides. |
| `prjxray/compare_exported.sh` | Export a routed database to a Vivado project, apply placement/routing constraints, run Vivado, and strictly compare the implemented placement and routing exports. |
| `prjxray/.run_random_test.sh` | Generate seeded random `pnr_tests` designs, check combinational chain length, and run the scalepnr-to-Vivado verification pipeline; defaults to 100 iterations. |
| `prjxray/.run_nextpnr_test.sh` | Compare scalepnr and nextpnr-xilinx PnR time and completion on shared generated/synthesized inputs, without Vivado. Defaults to ten designs per chain-length limit of 10, 20, and 30. |
| `prjxray/1`, `prjxray/2`, `prjxray/3` | Boardless integration examples for a processor core, memory controller, and video encoder. Run synthesis, scalepnr, and bitstream conversion without Vivado; see [README.examples.md](prjxray/README.examples.md). |

`prjxray/.prepare.sh` prepares external tools and databases. Exporters,
comparison scripts, profiling helpers, and log-cleanup utilities support the
flows above. Record the design seed, input files, resource limits, and output
directory when reporting a result. Keep generated logs, images, databases,
downloaded dependencies, and binaries out of source commits.
