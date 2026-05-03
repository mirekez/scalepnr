#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if ! yosys -p "clkbufmap -inpad IBUFG *clk*; synth_xilinx -flatten -arch xc7 -top test; write_json test.json" test.sv; then
    if [ ! -f test.json ]; then
        echo "yosys failed and test.json does not exist" >&2
        exit 1
    fi
    echo "yosys failed; reusing existing test.json" >&2
fi

(
    cd ../../build
    ./scalepnr ../tests/prjxray/test.tcl
)
