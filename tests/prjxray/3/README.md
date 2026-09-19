# StereoNinja TMDS encoder example

Run `./build.sh` (or `./build.sh --synth-only`). The wrapper exposes the original
encoder's pixel, control, sync, reset and encoded-word ports. Clock: 40 ns.
The upstream unused auxiliary-data input is tied low; its undriven `data_o`
output is unconnected. No ECP5 clock, camera or serializer macros are included.

See [shared instructions](../README.examples.md) for dependencies, pinned source
revision, output paths and the boardless-pinout warning.
