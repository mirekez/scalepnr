# quasiSoC CPU example

Run `./build.sh` (or `./build.sh --synth-only`). The original `riscv_multicyc`
top is built with RV32M/IRQ_EN enabled and all external ports.
The full SoC, board peripherals and firmware are not included. Clock: 16 ns.

See [shared instructions](../README.examples.md) for dependencies, pinned source
revision, output paths and the boardless-pinout warning.
