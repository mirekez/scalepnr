# UberDDR3 controller example

Run `./build.sh` (or `./build.sh --synth-only`). `top.v` exercises the original
controller with one byte lane and BIST, using a shift-input/indexed-output test
interface to keep package pin usage small. Clock: 10 ns. No DDR PHY is included;
this is not a bitstream for operating an external DRAM.

See [shared instructions](../README.examples.md) for dependencies, pinned source
revision, output paths and the boardless-pinout warning.
