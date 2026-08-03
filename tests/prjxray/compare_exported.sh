#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
design_db=${1:-"$script_dir/design_state.db"}
output_dir=${2:-"$script_dir/vivado_export_compare"}
db_dir=${SCALEPNR_DB_DIR:-"$script_dir/db"}
sv_source=${SCALEPNR_SV:-"$script_dir/test.sv"}
top=${SCALEPNR_TOP:-test}

if [[ -n ${VIVADO:-} ]]; then
    vivado_bin=$VIVADO
elif command -v vivado >/dev/null 2>&1; then
    vivado_bin=$(command -v vivado)
else
    vivado_bin=/home/me/Xilinx/Vivado/2024.2/bin/vivado
fi

for required in "$design_db" "$sv_source" "$script_dir/db2prj.py" "$script_dir/compare_pnr.py"; do
    if [[ ! -e $required ]]; then
        printf 'ERROR: required input does not exist: %s\n' "$required" >&2
        exit 2
    fi
done
if [[ ! -x $vivado_bin ]]; then
    printf 'ERROR: Vivado executable not found: %s\n' "$vivado_bin" >&2
    exit 2
fi

mkdir -p "$output_dir"
printf 'Exporting %s to %s\n' "$design_db" "$output_dir"
python3 "$script_dir/db2prj.py" \
    "$design_db" "$output_dir" \
    --db-dir "$db_dir" \
    --sv "$sv_source" \
    --top "$top" \
    >"$output_dir/db2prj.log" 2>&1

printf 'Running Vivado with every generated placement and routing constraint\n'
: >"$output_dir/vivado.stdout.log"
if ! (
    cd "$output_dir"
    "$vivado_bin" \
        -mode batch \
        -source create_project.tcl \
        -nojournal \
        -log create_project.tcl.log \
        >vivado.stdout.log 2>&1
); then
    tail -n 80 "$output_dir/create_project.tcl.log" >&2 || true
    exit 1
fi

printf 'Comparing packed scalepnr output with the implemented Vivado design\n'
python3 "$script_dir/compare_pnr.py" \
    --scalepnr "$output_dir/scalepnr_place_route_export.txt" \
    --vivado "$output_dir/place_route_export.txt" \
    --routing-tcl "$output_dir/routing.tcl" \
    --vivado-log "$output_dir/create_project.tcl.log" \
    --design-db "$design_db" \
    --top 10 \
    --strict
