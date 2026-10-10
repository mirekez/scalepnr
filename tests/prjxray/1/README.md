# quasiSoC CPU example

Run `./build.sh` (or `./build.sh --synth-only`). The original `riscv_multicyc`
top is built with RV32M/IRQ_EN enabled and all external ports.
The full SoC, board peripherals and firmware are not included. Clock: 16 ns.

The full build uses incremental PnR with a 1000-cell run target (half the
2000-cell profiling target). Override with `SCALEPNR_PNR_RUN_CELLS`.
Later runs prefer placement around less-used routing fabric; see
[the run contract](../../../doc/pnr.md).

See [shared instructions](../README.examples.md) for dependencies, pinned source
revision, output paths and the boardless-pinout warning.
