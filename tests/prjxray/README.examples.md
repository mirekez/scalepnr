# Vivado-free bitstream examples

Run from any directory:

```sh
tests/prjxray/1/build.sh
tests/prjxray/2/build.sh
tests/prjxray/3/build.sh
```

Each run executes **Yosys → scalepnr (place and route) → db2fasm →
fasm2frames → xc7frames2bit**. No Vivado, nextpnr, exported-design comparison,
or programmer is invoked. It uses the device-loading and routing commands in
`test.tcl`, the route-completion check from `pnr_compare.py`, and the conversion
tools used by `test.sh`.

| Directory | Original core exercised | Clock |
| --- | --- | --- |
| 1 | quasiSoC `riscv_multicyc`, including its register file, ALU, privilege and multiply/divide units | 16 ns |
| 2 | UberDDR3 `ddr3_controller`, one byte lane, BIST enabled, ECC/second Wishbone disabled | 10 ns |
| 3 | StereoNinjaFPGA `TMDS_Encoder` | 40 ns |

These are **core-level, boardless integration examples**, not builds of the
projects' complete board tops. Example 2 has a byte-shift test interface for
controller inputs and an indexed byte readout of outputs; it does not fake or
instantiate a DDR PHY. Example 3 omits the camera, PLLs and ECP5 serializer.
Upstream RTL is read directly and never rewritten. Memory, DSP, shift-register,
carry and wide-mux inference is disabled to use the current LUT/FF export flow.
`INV` is mapped to an equivalent `LUT1` because the exporter has no separate
inverter emitter. On newer Yosys versions, buffer-normalized aliases are converted
back to plain connections before writing JSON.

## Prerequisites

Build scalepnr with `cmake --build build --target scalepnr -j4`. Install working
Yosys and Python 3 (with Project X-Ray/FASM dependencies). The common resources
prepared by `.prepare.sh` must exist: `db/`, `prjxray/`, `prjxray-db/`, and
`prjxray/build/tools/xc7frames2bit`. `.prepare.sh` also prepares unrelated
comparison tools; the example runner neither calls it automatically nor uses
those tools.

Install just the Python bitstream dependencies locally (no system packages or
Vivado required):

```sh
python3 -m pip install --target tests/prjxray/.tools/bitstream-python \
    -r tests/prjxray/example_requirements.txt
```

The runner adds this ignored tool directory and the vendored Project X-Ray/FASM
sources to `PYTHONPATH`, and tests the converter's imports before synthesis.

All three use the existing **xc7a100tfgg676-1** database. Constraints are generated
from the synthesized ports and `db/package_pins.csv`: unique IOB33 pins,
LVCMOS33, and an MRCC clock pin. This arbitrary test pinout is **not a board
pinout; do not program a board with these files**. Producing a bitstream checks
tool integration, not hardware functionality or timing closure.

The source repositories already present in these directories are ignored by the
parent repository. On a fresh checkout, populate them explicitly:

```sh
git clone https://github.com/regymm/quasiSoC tests/prjxray/1/quasiSoC
git -C tests/prjxray/1/quasiSoC checkout 17918fc66b83610b8948af7d201dffb4cf30daf8
git clone https://github.com/AngeloJacobo/UberDDR3 tests/prjxray/2/UberDDR3
git -C tests/prjxray/2/UberDDR3 checkout a1258e2eedefa6fba1a425becaded83928c78709
git clone https://github.com/StereoNinja/StereoNinjaFPGA tests/prjxray/3/StereoNinjaFPGA
git -C tests/prjxray/3/StereoNinjaFPGA checkout 2def6f03fb93285817ced475b7c67dee756ac51a
```

## Outputs and failures

Each invocation creates a fresh `N/build/run.XXXXXXXX/`. It contains per-stage
logs, `design.json`, `constraints.tcl`, `design_state.db`, `design.fasm`,
`design_fasm.warnings`, `design.frm` and, **only if all stages succeed**,
`design.bit`. The runner prints the exact output directory. It never reuses old
netlists or placement checkpoints. Incomplete routing, unsupported primitives,
FASM conversion warnings, missing tools and timeouts fail the run; failures
remain available for investigation. No timing acceptance thresholds are changed.

Overrides: `YOSYS`, `PYTHON`, `SCALEPNR` (executable paths), and
`SCALEPNR_EXAMPLE_TIMEOUT` (seconds per non-PnR external stage, default 600).
PnR uses independent routing-stage budgets of 600 seconds, configurable with
`SCALEPNR_ROUTE_STAGE_TIMEOUT` and the router's individual stage overrides.
Loading and placement do not consume those routing budgets. The whole-PnR
external timeout is disabled by default so routing can hand off between stages
and render a terminal failure. Set `SCALEPNR_EXAMPLE_PNR_TIMEOUT` to a positive
number of seconds to opt into a whole-process cap (0 disables it).
`N/build.sh --synth-only` checks synthesis and pin constraints without running
PnR or producing a bitstream. For example:

```sh
YOSYS=/path/to/working/yosys tests/prjxray/3/build.sh --synth-only
python3 -m unittest discover -s tests/prjxray -p example_constraints_test.py -v
```

## Validation on 2026-09-18

Using Yosys 0.69+62 and a rebuilt Debug scalepnr:

| Example | Synthesized cells | Package port bits | Synthesis + constraints |
| --- | ---: | ---: | --- |
| 1 | 6,122 | 262 | Pass |
| 2 | 5,103 | 25 | Pass |
| 3 | 344 | 24 | Pass |

All eight constraint-generator regression tests pass. An injected scalepnr
failure also exits unsuccessfully without producing a bitstream.

The full example 3 attempt hit an explicit 600-second scalepnr timeout in
**Moving destinations**, after successful clock routing (43/43 sinks) and
more than 2,100 routing passes overall. Device subtype loading alone took
187 seconds. The log includes repeated relocation failures with
`no distinct free terminal path`, including generated MUXF7 passthrough sources.
No `.bit` was produced; end-to-end bitstream generation is **not yet verified**.
Examples 1 and 2 have not been run through full PnR.

The default `/snap/bin/yosys` on this machine fails at startup with a `dlerror`
symbol lookup error. Validation used:

```sh
export YOSYS=/home/me/cpphdl/build/tools/oss-cad-suite/bin/yosys
```

The failed full-flow log is retained locally at
`3/build/run.Bx7egpla/scalepnr.log`.

## Example 1 validation on 2026-09-19

The latest full run passed synthesis, placement, clock routing (2,191 sinks),
constant routing (2,286 tasks), and trunk recovery. It reached the unchanged
1,800-second PnR timeout after Fanouts pass 34, with 7,580 active tasks left.
No bitstream was produced. The log is retained locally at
`1/build/run.0IaCTR8a/scalepnr.log`.

A separate read-only audit reproduced all 125 completed pass reports exactly
and checked all 30,932 Tiles at that pass boundary. All 370,858 occupied nodes
had live owners or verified resource-pin reservations, with zero unexplained
occupancy, missing required leases, or stale/missing route registrations.
This verifies the occupancy bookkeeping at that snapshot, not complete routing.
The report and full proofs are retained locally in `1/build/fixed_audit.ukG4u8so/`.
