#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if ! yosys -p "clkbufmap -inpad IBUFG *clk*; synth_xilinx -flatten -arch xc7 -top test; write_json test.json; write_edif -pvector bra -attrprop test.edf" test.sv; then
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

python3 db2fasm.py design_state.db design.fasm --db-dir db --warnings design_fasm.warnings

if [ -f prjxray/utils/fasm2frames.py ] && [ -x prjxray/build/tools/xc7frames2bit ]; then
    export XRAY_DATABASE_DIR="$PWD/prjxray-db"
    export XRAY_DATABASE="artix7"
    export XRAY_PART="xc7a100tfgg676-1"
    export PYTHONPATH="$PWD:$PWD/prjxray:${PYTHONPATH:-}"
    python3 prjxray/utils/fasm2frames.py --sparse design.fasm design.frm
    prjxray/build/tools/xc7frames2bit \
        --part_file prjxray-db/artix7/xc7a100tfgg676-1/part.yaml \
        --part_name xc7a100tfgg676-1 \
        --frm_file design.frm \
        --output_file design.bit
fi
