#!/usr/bin/env bash
# Shared, Vivado-free RTL -> scalepnr -> Project X-Ray bitstream flow.
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/../.." && pwd)
if [[ $# -lt 1 || $# -gt 2 || ! $1 =~ ^[123]$ ||
      ( $# -eq 2 && $2 != --synth-only ) ]]; then
    echo "Usage: $0 {1|2|3} [--synth-only]" >&2
    exit 2
fi
example_dir=$script_dir/$1
synth_only=${2:-}
source "$example_dir/example.conf"
executable() {
    local path
    if ! path=$(command -v "$1"); then
        echo "Required executable not found: $1" >&2
        return 2
    fi
    # Preserve symlink names (notably /snap/bin/yosys), but survive later cd's.
    if [[ $path != /* ]]; then path=$PWD/$path; fi
    printf '%s\n' "$path"
}
yosys_bin=$(executable "${YOSYS:-yosys}")
python_bin=$(executable "${PYTHON:-python3}")
scalepnr_bin=${SCALEPNR:-$repo_dir/build/scalepnr}
stage_timeout=${SCALEPNR_EXAMPLE_TIMEOUT:-600}
pnr_timeout=${SCALEPNR_EXAMPLE_PNR_TIMEOUT:-0}
if [[ ! $stage_timeout =~ ^[1-9][0-9]*$ ]]; then
    echo 'SCALEPNR_EXAMPLE_TIMEOUT must be a positive number of seconds' >&2
    exit 2
fi
if [[ ! $pnr_timeout =~ ^(0|[1-9][0-9]*)$ ]]; then
    echo 'SCALEPNR_EXAMPLE_PNR_TIMEOUT must be zero (disabled) or a positive number of seconds' >&2
    exit 2
fi
# Routing has independent stage deadlines. Loading and placement must not
# consume their budget or kill PnR before its failure renderer can run.
export SCALEPNR_ROUTE_STAGE_TIMEOUT=${SCALEPNR_ROUTE_STAGE_TIMEOUT:-600}
command -v timeout >/dev/null
# Never use a stale JSON, placement checkpoint, or bitstream from another run.
mkdir -p "$example_dir/build"
run_dir=$(mktemp -d "$example_dir/build/run.XXXXXXXX")
stage=preflight
trap 'status=$?; if (( status != 0 )); then echo "FAIL: $stage (exit $status); logs: $run_dir" >&2; fi' EXIT
run_stage() {
    stage=$1
    shift
    local limit=$stage_timeout
    if [[ $stage == scalepnr ]]; then limit=$pnr_timeout; fi
    echo "[$stage] $run_dir/$stage.log (external timeout: ${limit}s; 0 disables)"
    if timeout --kill-after=15s "${limit}s" "$@" >"$run_dir/$stage.log" 2>&1; then
        return 0
    else
        local status=$?
        tail -n 40 "$run_dir/$stage.log" >&2
        return "$status"
    fi
}
export PYTHONPATH="$script_dir:$script_dir/.tools/bitstream-python:$script_dir/prjxray:$script_dir/prjxray/third_party/fasm${PYTHONPATH:+:$PYTHONPATH}"
if [[ -z $synth_only ]]; then
    scalepnr_bin=$(executable "$scalepnr_bin")
    for required in "$script_dir/db/tilegrid.json" "$script_dir/db/tileconn.json" \
        "$script_dir/db/package_pins.csv" \
        "$script_dir/prjxray-db/artix7/xc7a100tfgg676-1/part.yaml" \
        "$script_dir/prjxray/utils/fasm2frames.py"; do
        if [[ ! -s $required ]]; then
            echo "Missing $required; prepare the Project X-Ray tools/database first." >&2
            exit 2
        fi
    done
    if [[ ! -x $script_dir/prjxray/build/tools/xc7frames2bit ]]; then
        echo 'Missing prjxray/build/tools/xc7frames2bit' >&2
        exit 2
    fi
    if ! cmp -s "$script_dir/db/package_pins.csv" \
        "$script_dir/prjxray-db/artix7/xc7a100tfgg676-1/package_pins.csv" || \
       ! cmp -s "$script_dir/db/tilegrid.json" \
        "$script_dir/prjxray-db/artix7/xc7a100t/tilegrid.json"; then
        echo 'db/ must describe xc7a100tfgg676-1, matching the bitstream target.' >&2
        exit 2
    fi
    run_stage dependencies "$python_bin" "$script_dir/prjxray/utils/fasm2frames.py" --help
fi
cd "$example_dir"
run_stage yosys_capabilities "$yosys_bin" -Q -T -p 'help bufnorm'
normalize_buffers=
if [[ $(<"$run_dir/yosys_capabilities.log") == *'bufnorm [options]'* ]]; then
    normalize_buffers='bufnorm -conn;'
fi
synth_input=(-s synth.ys)
if [[ -f synth.tcl ]]; then
    synth_input=(-c synth.tcl)
fi
run_stage synthesis "$yosys_bin" -T "${synth_input[@]}" -p \
    "synth_xilinx -flatten -arch xc7 -nobram -nolutram -nosrl -nodsp -nocarry -nowidelut -top $top; techmap -map ../example_cells.v; $normalize_buffers delete t:\$scopeinfo; check -assert; write_json \"$run_dir/design.json\""
run_stage constraints "$python_bin" "$script_dir/example_constraints.py" \
    "$run_dir/design.json" "$top" "$clock_port" "$clock_period" \
    "$script_dir/db/package_pins.csv" "$run_dir/constraints.tcl"
if [[ -n $synth_only ]]; then
    echo "Synthesis and constraints OK: $run_dir (PnR/bitstream not run)"
    exit 0
fi
# test.tcl already loads the device, places, routes, and exports design state.
# Explicitly disable its debug/reuse shortcuts for this full-flow test.
export TCL_LIBRARY="$repo_dir/libs/tcl8.6.14/library"
export SCALEPNR_TEST_JSON="$run_dir/design.json" SCALEPNR_TEST_TOP="$top"
export SCALEPNR_TEST_CONSTRAINTS="$run_dir/constraints.tcl"
export SCALEPNR_DESIGN_DB="$run_dir/design_state.db" SCALEPNR_QUIET_ROUTES=1
unset SCALEPNR_PLACEMENT_DB SCALEPNR_CLOCK_ONLY SCALEPNR_SKIP_WRITE_DESIGN
export SCALEPNR_SKIP_READBACK=1
cd "$run_dir"
run_stage scalepnr "$scalepnr_bin" "$script_dir/test.tcl"
run_stage routing_check "$python_bin" "$script_dir/pnr_compare.py" \
    verify-scalepnr "$run_dir/scalepnr.log" --expect-clock
run_stage fasm "$python_bin" "$script_dir/db2fasm.py" \
    design_state.db design.fasm --db-dir "$script_dir/db" --warnings design_fasm.warnings
if [[ -s design_fasm.warnings ]]; then
    echo 'FASM export reported warnings; refusing to silently omit configuration:' >&2
    head -n 30 design_fasm.warnings >&2
    exit 1
fi
run_stage frames "$python_bin" "$script_dir/prjxray/utils/fasm2frames.py" \
    --db-root "$script_dir/prjxray-db/artix7" --part xc7a100tfgg676-1 \
    design.fasm design.frm
run_stage bitstream "$script_dir/prjxray/build/tools/xc7frames2bit" \
    --part_file "$script_dir/prjxray-db/artix7/xc7a100tfgg676-1/part.yaml" \
    --part_name xc7a100tfgg676-1 --frm_file design.frm --output_file design.bit.tmp
test -s design.bit.tmp
mv design.bit.tmp design.bit
echo "Built: $run_dir/design.bit"
