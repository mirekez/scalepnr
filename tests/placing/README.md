# Synthesized placement acceptance tests

## Two-clock variants

Each size also contains a `2clk/` directory with the ten original families and
an additional `async_fifo.sv`. The count target applies to the entire design,
including both clock domains. The original single-clock cases are retained.

The ten variants split useful processing sections between independent 16-bit
input streams, clocked by `clk_a` and `clk_b`, and export both 16-bit results.
`clk_a` uses the family's period in the table below; `clk_b` uses **1.5 times**
that period. The map/reduce variant registers its incoming samples before the
multiplier map. Both clocks must have real register endpoints after synthesis.

The additional asynchronous FIFO uses 8-entry, 12-bit banks, registered Gray
pointers, two-stage pointer synchronizers, full/empty gating, and a registered
XOR reduction of bank outputs. It writes at **8 ns** and reads at **12 ns**.
Explicit maximum data-path limits are **6 ns from `clk_a` to `clk_b`** and
**7 ns in the reverse direction**, including mapped memory-data paths and
pointer crossings. These limits remain below the faster clock's period.
Gray-pointer CDCs need physical timing bounds; see the primary
[dual-clock FIFO constraint guidance](https://docs.altera.com/r/docs/683082/25.1.1/quartus-prime-pro-edition-user-guide-design-recommendations/dual-clock-fifo-timing-constraints?contentId=G0iC0gBaqgpYEJSlKkEaXA).
The RTL and fixture do not instantiate that vendor's primitives.

Each `.sv` has matching `.json` constraints consumed by the fixture and a `.tcl`
file usable with ScalePNR after loading the design. CTest verifies their agreement
and records their hashes. The Tcl file uses `create_clock` and, for the FIFO,
`set_max_delay -datapath_only`. No asynchronous clock-group cuts hide FIFO paths.
See [clock constraint semantics](../../doc/clocks.md).

Every chunked placement stage checks endpoint counts and WNS/TNS **per capture clock**. At the end,
an independent evaluator reconstructs physical delays from **every launch
domain** in each endpoint cone, including branches that are not globally
critical. Every declared crossing direction must have exercised paths, satisfy
its own limit within the same 0.10 ns actionable tolerance, and agree with the
production timing evaluator. Missing clocks, lost endpoints, unintended
crossings and stale constraint files fail the test.

```sh
ctest --test-dir build-release -j12 -R '^placing\.synth\.1k\.2clk\.' --output-on-failure
ctest --test-dir build-release -j12 -L 2clk --output-on-failure
python3 tests/placing/generate_2clk.py 1k --yosys /path/to/yosys
python3 tests/placing/report.py --two-clocks --build build-release \
  --output tests/placing/validation_2clk.json --markdown tests/placing/RESULTS_2CLK.md
```

The two-clock generator calibrates up to four independent families concurrently;
use `--jobs 1` on smaller machines. It generates only the requested size tier.

When Icarus Verilog (`iverilog` and `vvp`) is installed, CMake also registers
`placing.fifo_functional`. It simulates the generated FIFO RTL at different
clock periods/phases, checking ordering, overflow/underflow, repeated pointer
wrap and full/empty coverage. Placement timing is still a generic estimate;
this is not routed CDC or metastability signoff. Hardware reset sequencing,
clock-to-Q, setup/hold and bus-skew analysis need a device-specific flow.

See [two-clock results](RESULTS_2CLK.md), [development plan](PLAN_2CLK.md) and
[problems found and fixed](PROBLEMS_2CLK.md).

## Single-clock families and shared acceptance criteria

Ten **standalone SystemVerilog designs per size** target 1k, 5k, 10k, 30k,
50k, 80k and 100k synthesized cells. The count means post-optimization `$lut`
and one-bit flip-flop cells, excluding test I/O and constants. Each manifest
records measured counts, generic primitive mix, generator parameters and Yosys
version. The allowed count error is **±10%**, checked again on every test run.

| Family / filename | Processing and data flow | Clock |
| --- | --- | --- |
| `arithmetic_pipeline.sv` | Registered add, rotate and XOR stages; feed-forward stream | 5 ns / 200 MHz |
| `systolic_fir.sv` | Local delayed taps, constant products, pipelined sums | 6 ns / 167 MHz |
| `map_reduce.sv` | Registered distribution, 8×8 multiply map, balanced registered reduction | 8 ns / 125 MHz |
| `ram_lookup.sv` | 16×16 synchronous banks; cyclic addresses and read/modify/write | 10 ns / 100 MHz |
| `fifo_stream.sv` | 8×16 circular queues, occupancy, local enables and pointer feedback | 8 ns / 125 MHz |
| `crossbar_switch.sv` | Four-way registered permutation, data-dependent selects | 6 ns / 167 MHz |
| `crc_feedback.sv` | Byte-wise CRC-16 polynomial feedback and stream forwarding | 5 ns / 200 MHz |
| `stencil_mesh.sv` | Registered five-point, two-dimensional neighbor exchange | 6 ns / 167 MHz |
| `priority_arbiter.sv` | Priority grant with rotating eligibility mask | 8 ns / 125 MHz |
| `compare_exchange.sv` | Cascaded registered unsigned compare/exchange sections | 6 ns / 167 MHz |

These are placement workloads, not application-level functional verification
suites. Scaling repeats useful processing/storage sections; there is no `keep`
padding or vendor primitive instantiation. All logic contributes to an output.
RAM and multipliers map to **soft logic**, using default Yosys synthesis plus
`abc -lut 4`. Thus memory and arithmetic connectivity are exercised without
assuming a vendor's hard RAM/DSP geometry. The C++ fixture imports every mapped
cell and net; it does not replace RTL with a generated random placement puzzle.
Original Yosys types remain in cell attributes. Generic FF variants retain their
clock, enable and reset inputs and share the existing abstract FD resource class.

## Run

Activate the project's compiler environment. Install Yosys there if needed
(`conda install -p ./.conda -c conda-forge yosys`), then:

```sh
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --target placing_synth_test -j4
ctest --test-dir build-release -R '^placing\.synth\.1k\.' --output-on-failure
ctest --test-dir build-release -j8 -R '^placing\.synth\.' --output-on-failure
```

`-DSCALEPNR_YOSYS=/absolute/path/to/yosys` selects an existing installation.
CTest's `placing.synth.yosys` setup fixture runs first, including when a size is
selected with `-R`. Each design then synthesizes its `.sv` afresh. Tests are
isolated by working directory and reserve two CTest processor slots each.
`-j8` runs up to four designs concurrently; omit it for serial validation on
smaller machines. Development still validates one size tier before generating
the next. Disable this optional suite with `-DSCALEPNR_SYNTH_PLACING_TESTS=OFF`.
The recorded full validation took about 46 minutes with six designs running
concurrently (`-j12`); individual size tiers are much quicker.

Artifacts live under `build-release/tests/placing/<size>/<family>/`:
`synth.ys`, `synthesis.log`, `netlist.json`, `synthesis.json`, `placement.log`, and
`metrics.json`. Metrics include every chunk's packing/Sorting/Swapping timing,
violations, deferred edges and elapsed stage intervals, plus final setup results.
Any conditional final Sorting pass is recorded separately as `post_swap_sorting`.
Intervals include orchestration/audit work; they are not isolated performance
benchmarks. Production stage summaries and pre-smearing work counts are also
retained in `metrics.json`.

## Model and acceptance

The abstract square FPGA provides 16 general LUT, 16 small LUT and 16 register
positions per tile. The most-used resource determines the device size at a
nominal 65% occupancy; each resource is checked independently. Outline's scalar
LUT budget counts general LUT sites, since inverter slots cannot replace them. Inputs and outputs
are fixed on opposite edges: inputs (including clocks) at `x=0`, outputs at
`x=side-1`, and `y=pin_index % side`, with additional I/O slots when needed.
The square grid is sized as
`side=max(4, ceil(sqrt(max(general_LUTs, one_input_LUTs, registers)/(16*0.65))))`.
For example, the 100k two-clock FIFO uses 72×72 tiles and its stencil uses
93×93 tiles. Exact per-design dimensions are in `validation_2clk.json`.
Outline's normalized 10×10 coordinate space is scaled onto this physical tile
grid; it is not a fixed 10×10 device. No connectivity-derived placement hints
are given.

The suite uses the existing default intrinsic LUT delays (plus 0.05 ns for
one-input LUTs, matching the inverter model) and Manhattan wire
calibration (0.010 ns local, 0.035/0.040 ns per horizontal/vertical tile,
0.005 ns bend, and 0.002 ns × log₂(fanout)). Clock periods are fixed **by family,
not by size**. This is an architecture-independent setup estimate, not routed
silicon Fmax: the current timing engine lacks clock-to-Q, setup/hold cell arcs,
clock-tree skew and package input/output delays. There is one ideal primary clock;
register data/control inputs driven by real nets must all be constrained.

A test passes only when:

1. Yosys `check -assert` passes, the cell mix includes LUTs and FFs, and the mapped
   count is within ±10% of the folder name. Unknown primitives and undefined net
   connections are errors, not silently omitted cells.
2. Estimate covers every core cell. Outline executes **once over the entire
   design**, before chunk flags are set; its coarse capacity phase must execute.
   All core coordinates are finite and
   within its 10×10 continuous mesh. Outline must not assign core resources.
3. The production planner admits approximately `SCALEPNR_PNR_RUN_CELLS` cells
   (default 1000) per run, retaining indivisible bunches and connected shared
   COMB networks. The temporary run view coalesces their owners for coherent
   packing and restores Estimate's original tree afterward. Additional capture
   registers stay with shared outputs when that fits the run target; launching
   registers still separate calculations. Every run executes
   pre-smearing, exact packing/refinement, Sorting and Swapping in order.
   If accepted swaps leave an actionable deficit, one final Sorting pass uses
   the changed geometry before the run is frozen. Its `post_swap_sorting` stage
   receives the same legality, fixed-cell and endpoint audits.
4. After every stage, admitted cells have existing, occupied, collision-free
   positions and matching tile coordinates. Pending core cells remain unplaced;
   boundary I/O and completed runs never move.
5. All expected setup endpoints survive every stage; all metrics remain finite.
   The final graph has **zero unplaced edges** and **WNS ≥ −0.10 ns**. This tolerance
   matches the placement optimizer's actionable-slack threshold. During chunked
   placement, pending cells use their global Outline targets for estimated wire
   delay while remaining counted as unplaced. Routing is not
   run in this suite; production routing regressions remain separate.

Sorting/Swapping may correctly do no work when the declared clock already
closes. Existing focused `placing.sorting`/`placing.swapping` regressions check
repair behavior under forced violations; this suite checks realistic RTL flows.

## Regeneration and development order

Run `python3 tests/placing/generate.py 1k --yosys /path/to/yosys` to regenerate
**one size only**. Run its CTests and investigate failures before generating the
next size. Larger tiers start from the preceding manifest's calibration, then
resynthesize until they hit the count tolerance. Reconfigure CMake after adding
a tier. Yosys/ABC versions can change mapped cell counts; manifests record the
version used for calibration. See [RESULTS.md](RESULTS.md) for the validation
summary, [validation.json](validation.json) for individual measurements, and
[PLAN.md](PLAN.md) / [PROBLEMS.md](PROBLEMS.md) for development and repairs.
Regenerate that report with `report.py --build build-release --sizes 1k 5k 10k
30k 50k 80k 100k --output tests/placing/validation.json
--markdown tests/placing/RESULTS.md`; it refuses stale RTL
hashes, stale placer binaries or clock requirements, missing measurements, or failing results.
