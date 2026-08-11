#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/../.." && pwd)

usage() {
    cat <<'EOF'
Usage: .run_random_test.sh SIZE [SEED]

Generate and verify 100 random pnr_tests pipeline designs. SIZE is the
number of randomly selected processing stages in each generated pipeline.

Environment overrides:
  SCALEPNR_RANDOM_ITERATIONS       iteration count (default: 100)
  SCALEPNR_RANDOM_SEED             first iteration seed (overridden by SEED)
  SCALEPNR_RANDOM_COMPLEXITY       pnr_tests node complexity (default: 1)
  SCALEPNR_RANDOM_MAX_WIDTH        maximum generated datapath width (default: 64)
  SCALEPNR_RANDOM_MAX_CHAIN_LENGTH maximum combinational cells between registers (default: 30)
  SCALEPNR_RANDOM_STAGE_TIMEOUT    seconds allowed per external stage (default: 1800)
  SCALEPNR_RANDOM_MEMORY_KB        scalepnr virtual-memory limit in KiB
                                   (default: 6291456; use 0 to disable)
  SCALEPNR_RANDOM_KEEP_SUCCESS     keep successful iteration artifacts when nonempty
  SCALA_CLI                        scala-cli executable
  VIVADO                           Vivado executable
EOF
}

if [[ $# -lt 1 || $# -gt 2 || ! $1 =~ ^[1-9][0-9]*$ ||
      ( $# -eq 2 && ! $2 =~ ^[0-9]+$ ) ]]; then
    usage >&2
    exit 2
fi

size=$1
if [[ $# -eq 2 ]]; then
    base_seed=$2
elif [[ -n ${SCALEPNR_RANDOM_SEED:-} ]]; then
    base_seed=$SCALEPNR_RANDOM_SEED
else
    base_seed=$(od -An -N4 -tu4 /dev/urandom | tr -d ' ')
fi
if [[ ! $base_seed =~ ^[0-9]+$ ]]; then
    printf 'ERROR: seed must be a non-negative integer: %s\n' "$base_seed" >&2
    exit 2
fi
# Keep seeds in a range handled identically by Bash, Java, and Scala.
base_seed=$((base_seed % 2147483647))
iterations=${SCALEPNR_RANDOM_ITERATIONS:-100}
complexity=${SCALEPNR_RANDOM_COMPLEXITY:-1}
max_width=${SCALEPNR_RANDOM_MAX_WIDTH:-64}
max_chain_length=${SCALEPNR_RANDOM_MAX_CHAIN_LENGTH:-30}
stage_timeout=${SCALEPNR_RANDOM_STAGE_TIMEOUT:-1800}
pnr_memory_kb=${SCALEPNR_RANDOM_MEMORY_KB:-6291456}
part=xc7a100tfgg676-1
top=TestPipeline
pnr_tests_dir=$script_dir/pnr_tests
scala_cli=${SCALA_CLI:-$script_dir/.tools/scala-cli}
scalepnr=$repo_dir/build/scalepnr
db_dir=$script_dir/db
export TCL_LIBRARY=$repo_dir/libs/tcl8.6.14/library

for value_name in iterations complexity max_width max_chain_length stage_timeout pnr_memory_kb; do
    value=${!value_name}
    if [[ ! $value =~ ^[0-9]+$ ]] ||
       [[ $value_name != complexity && $value_name != pnr_memory_kb && $value -eq 0 ]]; then
        printf 'ERROR: %s must be a positive integer (complexity and memory may be zero): %s\n' "$value_name" "$value" >&2
        exit 2
    fi
done

if [[ ! -x $scala_cli ]] && command -v scala-cli >/dev/null 2>&1; then
    scala_cli=$(command -v scala-cli)
fi

if ! command -v java >/dev/null 2>&1; then
    for java_home in "${HOME}"/Xilinx/Vivado/*/tps/lnx64/jre* /opt/Xilinx/Vivado/*/tps/lnx64/jre*; do
        if [[ -x $java_home/bin/java ]]; then
            export JAVA_HOME=$java_home
            export PATH=$JAVA_HOME/bin:$PATH
            break
        fi
    done
fi
scala_jvm_args=()
if command -v java >/dev/null 2>&1; then
    scala_jvm_args=(--jvm system)
fi

required=(
    "$scala_cli"
    "$scalepnr"
    "$db_dir/tilegrid.json"
    "$db_dir/tileconn.json"
    "$db_dir/package_pins.csv"
    "$pnr_tests_dir/tests/pipeline/TestPipeline.scala"
    "$script_dir/random_pipeline.scala"
    "$script_dir/check_chain_length.py"
    "$script_dir/test.tcl"
    "$script_dir/compare_exported.sh"
)
for path in "${required[@]}"; do
    if [[ ! -e $path ]]; then
        printf 'ERROR: required random-test input is missing: %s\n' "$path" >&2
        printf 'Run %s first.\n' "$script_dir/prepare.sh" >&2
        exit 2
    fi
done

pnr_sources=(
    "$pnr_tests_dir/queue/NodeQueue.scala"
    "$pnr_tests_dir/mux/NodeMux.scala"
    "$pnr_tests_dir/mux/NodeDemux.scala"
    "$pnr_tests_dir/math/NodeMul.scala"
    "$pnr_tests_dir/math/NodeDiv.scala"
    "$pnr_tests_dir/decode/NodeMap.scala"
    "$pnr_tests_dir/memory/NodeMemory.scala"
    "$pnr_tests_dir/common/AttributeAnnotation.scala"
    "$pnr_tests_dir/tests/Crossbar.scala"
    "$pnr_tests_dir/tests/NodeFabric.scala"
    "$pnr_tests_dir/tests/XDCGen.scala"
    "$pnr_tests_dir/tests/pipeline/TestPipeline.scala"
    "$script_dir/random_pipeline.scala"
)

vivado_bin=${VIVADO:-}
if [[ -z $vivado_bin ]]; then
    if command -v vivado >/dev/null 2>&1; then
        vivado_bin=$(command -v vivado)
    else
        vivado_bin=/home/me/Xilinx/Vivado/2024.2/bin/vivado
    fi
fi
if [[ ! -x $vivado_bin ]]; then
    printf 'ERROR: Vivado executable not found: %s\n' "$vivado_bin" >&2
    exit 2
fi

run_stamp=$(date +%Y%m%d_%H%M%S)
run_root=$script_dir/random_runs/size_${size}_chain_${max_chain_length}_seed_${base_seed}_${run_stamp}_${BASHPID}
summary=$run_root/summary.tsv
mkdir -p "$run_root"
printf 'iteration\tseed\tmax_chain\tmeasured_chain\tsv_sha256\tstatus\tfailure_stage\tgenerate_s\tsynthesis_s\tchain_check_s\tscalepnr_s\tvivado_compare_s\n' >"$summary"

current_iteration=0
current_stage=initialization
current_log=
current_seed=$base_seed
last_stage_seconds=0

report_failure() {
    local status=$1
    local sv_hash=-
    if [[ -s ${work_dir:-}/generated/TestPipeline.sv ]]; then
        sv_hash=$(sha256sum "$work_dir/generated/TestPipeline.sv" | awk '{print $1}')
    fi
    case $current_stage in
        generate) generate_seconds=$last_stage_seconds ;;
        synthesis) synthesis_seconds=$last_stage_seconds ;;
        chain-check) chain_check_seconds=$last_stage_seconds ;;
        scalepnr) scalepnr_seconds=$last_stage_seconds ;;
        vivado-compare) vivado_seconds=$last_stage_seconds ;;
    esac
    if [[ -s ${chain_result:-} ]]; then
        measured_chain=$(<"$chain_result")
    fi
    printf '%d\t%d\t%d\t%s\t%s\tFAIL\t%s\t%d\t%d\t%d\t%d\t%d\n' \
        "$current_iteration" "$current_seed" "$max_chain_length" "$measured_chain" \
        "$sv_hash" "$current_stage" "$generate_seconds" "$synthesis_seconds" \
        "$chain_check_seconds" "$scalepnr_seconds" \
        "$vivado_seconds" >>"$summary"
    printf '\nERROR: random PnR iteration %d/%d (size=%d, seed=%d) failed during %s (status=%d).\n' \
        "$current_iteration" "$iterations" "$size" "$current_seed" "$current_stage" "$status" >&2
    printf 'Failure artifacts: %s\n' "$work_dir" >&2
    printf 'Replay: SCALEPNR_RANDOM_ITERATIONS=1 SCALEPNR_RANDOM_MAX_CHAIN_LENGTH=%q %q %q %q\n' \
        "$max_chain_length" "$script_dir/.run_random_test.sh" "$size" "$current_seed" >&2
    if [[ -n $current_log && -f $current_log ]]; then
        printf '%s\n' '---------------- log tail ----------------' >&2
        tail -n 120 "$current_log" >&2 || true
        printf '%s\n' '------------------------------------------' >&2
    fi
    exit "$status"
}

run_stage() {
    current_stage=$1
    current_log=$2
    shift 2
    printf '[%03d/%03d seed=%d] %-18s' \
        "$current_iteration" "$iterations" "$current_seed" "$current_stage"
    printf 'random_test_iteration=%d\nrandom_test_seed=%d\nrandom_test_stage=%s\n' \
        "$current_iteration" "$current_seed" "$current_stage" >"$current_log"
    local started=$SECONDS
    set +e
    if [[ $current_stage == scalepnr && $pnr_memory_kb -ne 0 ]]; then
        timeout --foreground --kill-after=15s "$stage_timeout" \
            bash -c 'ulimit -v "$1"; shift; exec "$@"' \
            _ "$pnr_memory_kb" "$@" >>"$current_log" 2>&1
    else
        timeout --foreground --kill-after=15s "$stage_timeout" \
            "$@" >>"$current_log" 2>&1
    fi
    local status=$?
    set -e
    last_stage_seconds=$((SECONDS - started))
    if ((status != 0)); then
        printf ' FAILED (%ds)\n' "$last_stage_seconds"
        report_failure "$status"
    fi
    printf ' OK (%ds)\n' "$last_stage_seconds"
}

printf 'Random PnR test: iterations=%d size=%d first_seed=%d complexity=%d max_width=%d max_chain=%d\n' \
    "$iterations" "$size" "$base_seed" "$complexity" "$max_width" "$max_chain_length"
printf 'Artifacts root: %s\n' "$run_root"

for ((iteration = 1; iteration <= iterations; ++iteration)); do
    current_iteration=$iteration
    current_seed=$(((base_seed + iteration - 1) % 2147483647))
    generate_seconds=0
    synthesis_seconds=0
    chain_check_seconds=0
    scalepnr_seconds=0
    vivado_seconds=0
    measured_chain=-
    work_dir=$run_root/iteration_$(printf '%03d' "$iteration")
    generated_dir=$work_dir/generated
    vivado_dir=$work_dir/vivado_export
    mkdir -p "$generated_dir"
    printf '%d\n' "$current_seed" >"$work_dir/seed.txt"

    run_stage generate "$work_dir/generate.log" \
        flock "$script_dir/.tools/random_generator.lock" \
        "$scala_cli" run "${pnr_sources[@]}" \
        --server=false "${scala_jvm_args[@]}" \
        --main-class ScalepnrRandomPipeline -- \
        "$generated_dir" random_design "$part" "$db_dir" "$complexity" "$size" \
        "$max_width" "$max_chain_length" "$current_seed"
    generate_seconds=$last_stage_seconds

    sv_file=$generated_dir/TestPipeline.sv
    raw_xdc=$generated_dir/random_design.xdc
    constraints=$generated_dir/scalepnr_constraints.tcl
    if [[ ! -s $sv_file || ! -s $raw_xdc ]]; then
        current_stage=generator-output
        current_log=$work_dir/generate.log
        report_failure 1
    fi

    # pnr_tests emits unquoted bus ports. Brace every port so Tcl keeps [N]
    # as part of the logical name. Clock creation is decided after synthesis.
    python3 - "$raw_xdc" "$constraints" <<'PY'
from pathlib import Path
import re
import sys

source = Path(sys.argv[1])
target = Path(sys.argv[2])
lines = []
pattern = re.compile(r"\[get_ports (.+)\]\s*$")
for raw_line in source.read_text().splitlines():
    match = pattern.search(raw_line)
    if match:
        raw_line = raw_line[:match.start()] + "[get_ports {" + match.group(1) + "}]"
    lines.append(raw_line)
target.write_text("\n".join(lines) + "\n")
PY

    run_stage synthesis "$work_dir/yosys.log" \
        bash -c 'cd "$1" && exec yosys -p "read_verilog -sv TestPipeline.sv; clkbufmap -inpad IBUFG *clock*; synth_xilinx -nobram -nolutram -nodsp -nocarry -flatten -arch xc7 -top TestPipeline; delete t:\$scopeinfo; write_json test.json; write_edif -pvector bra -attrprop test.edf"' \
        _ "$generated_dir"
    synthesis_seconds=$last_stage_seconds

    design_json=$generated_dir/test.json
    edif_file=$generated_dir/test.edf
    chain_result=$work_dir/chain_length.txt
    run_stage chain-check "$work_dir/chain_check.log" \
        python3 "$script_dir/check_chain_length.py" "$design_json" \
        --top "$top" --maximum "$max_chain_length" --result "$chain_result"
    chain_check_seconds=$last_stage_seconds
    measured_chain=$(<"$chain_result")

    python3 - "$design_json" "$constraints" <<'PY'
from pathlib import Path
import json
import sys

design = json.loads(Path(sys.argv[1]).read_text())
has_sequential = any(
    str(cell.get("type", "")).startswith(("FD", "LD"))
    for module in design.get("modules", {}).values()
    for cell in module.get("cells", {}).values()
)
if has_sequential:
    path = Path(sys.argv[2])
    path.write_text(
        "create_clock -name clock -period 5.0 [get_ports {clock}]\n"
        + path.read_text()
    )
PY

    design_db=$work_dir/design_state.db
    run_stage scalepnr "$work_dir/scalepnr.log" \
        env \
        SCALEPNR_TEST_JSON="$design_json" \
        SCALEPNR_TEST_TOP="$top" \
        SCALEPNR_TEST_CONSTRAINTS="$constraints" \
        SCALEPNR_DESIGN_DB="$design_db" \
        SCALEPNR_QUIET_ROUTES=1 \
        SCALEPNR_SKIP_READBACK=1 \
        "$scalepnr" "$script_dir/test.tcl"
    scalepnr_seconds=$last_stage_seconds
    if [[ ! -s $design_db ]]; then
        current_stage=scalepnr-database
        current_log=$work_dir/scalepnr.log
        report_failure 1
    fi

    run_stage vivado-compare "$work_dir/vivado_compare.log" \
        env \
        SCALEPNR_DB_DIR="$db_dir" \
        SCALEPNR_EDIF="$edif_file" \
        SCALEPNR_SV="$sv_file" \
        SCALEPNR_TOP="$top" \
        VIVADO="$vivado_bin" \
        "$script_dir/compare_exported.sh" "$design_db" "$vivado_dir"
    vivado_seconds=$last_stage_seconds

    sv_hash=$(sha256sum "$sv_file" | awk '{print $1}')
    printf '%d\t%d\t%d\t%s\t%s\tPASS\t-\t%d\t%d\t%d\t%d\t%d\n' \
        "$iteration" "$current_seed" "$max_chain_length" "$measured_chain" \
        "$sv_hash" "$generate_seconds" "$synthesis_seconds" \
        "$chain_check_seconds" "$scalepnr_seconds" "$vivado_seconds" >>"$summary"

    if [[ -z ${SCALEPNR_RANDOM_KEEP_SUCCESS:-} ]]; then
        rm -rf -- "$work_dir"
    fi
done

printf '\nPASS: all %d random designs of size %d were accepted and reproduced by Vivado.\n' \
    "$iterations" "$size"
printf 'Summary: %s\n' "$summary"
