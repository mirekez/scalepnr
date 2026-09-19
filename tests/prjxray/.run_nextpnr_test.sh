#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/../.." && pwd)

usage() {
    cat <<'EOF'
Usage: .run_nextpnr_test.sh SIZE [SEED]

Generate each random design once, synthesize it once, and run paired PnR
measurements with scalepnr and nextpnr-xilinx. SIZE is the number of random
pnr_tests stages in the generated pipeline.

Environment overrides:
  SCALEPNR_COMPARE_ITERATIONS    designs per chain limit (default: 10)
  SCALEPNR_COMPARE_CHAIN_LENGTHS space-separated limits (default: "10 20 30")
  SCALEPNR_COMPARE_CASES_PER_CHAIN optional space-separated case counts matching
                                 the chain limits (for example: "4 3 3")
  SCALEPNR_COMPARE_TIMEOUT       PnR timeout in seconds (default: 600)
  SCALEPNR_COMPARE_SCALEPNR_TIMEOUT complete scalepnr process timeout in
                                 seconds, including database loading and
                                 placement (default: 2400)
  SCALEPNR_COMPARE_ROUTE_STAGE_TIMEOUT scalepnr timeout for each routing stage
                                 in seconds (default: 1200)
  SCALEPNR_COMPARE_MEMORY_KB     virtual-memory limit per PnR process in KiB
                                 (default: 6291456; use 0 to disable)
  SCALEPNR_COMPARE_SEED          first seed (overridden by SEED)
  SCALEPNR_COMPARE_COMPLEXITY    pnr_tests complexity (default: 1)
  SCALEPNR_COMPARE_MAX_WIDTH     maximum datapath width (default: 64)
  SCALEPNR_COMPARE_FREQ_MHZ      target frequency (default: 200)
  SCALEPNR_COMPARE_MIN_CELLS     requested minimum synthesized cell count;
                                 stage count adapts until reached (default: 0)
  SCALEPNR_COMPARE_MAX_CELLS     optional maximum synthesized cell count;
                                 stage count adapts down when exceeded
  SCALEPNR_COMPARE_MAX_CANDIDATES maximum generated seeds per chain limit
                                 (default: 100 times the requested iterations)
  SCALEPNR_COMPARE_RUN_PARENT     parent directory for generated campaign data
                                 (default: tests/prjxray/nextpnr_runs)
  SCALEPNR_COMPARE_FAIL_FAST     stop on first PnR failure when nonempty
  SCALEPNR_COMPARE_STOP_ON_SCALEPNR_TIMEOUT stop immediately after the first
                                 scalepnr stage or process timeout
  SCALEPNR_SHARED_TIMEOUT        generation/synthesis timeout (default: 1800)
  SCALA_CLI                     scala-cli executable
  NEXTPNR_XILINX                nextpnr-xilinx executable
  NEXTPNR_XILINX_CHIPDB         xc7a100t chip database
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
elif [[ -n ${SCALEPNR_COMPARE_SEED:-} ]]; then
    base_seed=$SCALEPNR_COMPARE_SEED
else
    base_seed=$(od -An -N4 -tu4 /dev/urandom | tr -d ' ')
fi
base_seed=$((base_seed % 2147483647))

iterations=${SCALEPNR_COMPARE_ITERATIONS:-10}
chain_lengths=${SCALEPNR_COMPARE_CHAIN_LENGTHS:-"10 20 30"}
cases_per_chain=${SCALEPNR_COMPARE_CASES_PER_CHAIN:-}
pnr_timeout=${SCALEPNR_COMPARE_TIMEOUT:-600}
scalepnr_timeout=${SCALEPNR_COMPARE_SCALEPNR_TIMEOUT:-2400}
route_stage_timeout=${SCALEPNR_COMPARE_ROUTE_STAGE_TIMEOUT:-1200}
pnr_memory_kb=${SCALEPNR_COMPARE_MEMORY_KB:-6291456}
shared_timeout=${SCALEPNR_SHARED_TIMEOUT:-1800}
complexity=${SCALEPNR_COMPARE_COMPLEXITY:-1}
max_width=${SCALEPNR_COMPARE_MAX_WIDTH:-64}
freq_mhz=${SCALEPNR_COMPARE_FREQ_MHZ:-200}
min_cells=${SCALEPNR_COMPARE_MIN_CELLS:-0}
max_cells=${SCALEPNR_COMPARE_MAX_CELLS:-0}
max_candidates=${SCALEPNR_COMPARE_MAX_CANDIDATES:-$((iterations * 100))}
part=xc7a100tfgg676-1
top=TestPipeline

scala_cli=${SCALA_CLI:-$script_dir/.tools/scala-cli}
scalepnr=$repo_dir/build/scalepnr
nextpnr=${NEXTPNR_XILINX:-$script_dir/nextpnr-xilinx/build-compare/nextpnr-xilinx}
chipdb=${NEXTPNR_XILINX_CHIPDB:-$script_dir/nextpnr-xilinx/xilinx/xc7a100t.bin}
db_dir=$script_dir/db
pnr_tests_dir=$script_dir/pnr_tests
helper=$script_dir/pnr_compare.py
export TCL_LIBRARY=$repo_dir/libs/tcl8.6.14/library

for value_name in iterations pnr_timeout scalepnr_timeout route_stage_timeout pnr_memory_kb shared_timeout complexity max_width freq_mhz min_cells max_cells max_candidates; do
    value=${!value_name}
    if [[ ! $value =~ ^[0-9]+$ ]] ||
       [[ $value_name != complexity && $value_name != min_cells &&
          $value_name != pnr_memory_kb &&
          $value_name != max_cells && $value -eq 0 ]]; then
        printf 'ERROR: invalid %s value: %s\n' "$value_name" "$value" >&2
        exit 2
    fi
done
if ((max_cells != 0 && max_cells < min_cells)); then
    printf 'ERROR: maximum cell count %d is below minimum %d\n' \
        "$max_cells" "$min_cells" >&2
    exit 2
fi

read -r -a chain_caps <<<"$chain_lengths"
if ((${#chain_caps[@]} == 0)); then
    printf 'ERROR: no chain-length limits were provided\n' >&2
    exit 2
fi
for chain_cap in "${chain_caps[@]}"; do
    if [[ ! $chain_cap =~ ^[1-9][0-9]*$ ]]; then
        printf 'ERROR: invalid chain-length limit: %s\n' "$chain_cap" >&2
        exit 2
    fi
done
case_counts=()
if [[ -n $cases_per_chain ]]; then
    read -r -a case_counts <<<"$cases_per_chain"
    if ((${#case_counts[@]} != ${#chain_caps[@]})); then
        printf 'ERROR: case-count list must match the %d chain limits\n' \
            "${#chain_caps[@]}" >&2
        exit 2
    fi
else
    for _ in "${chain_caps[@]}"; do
        case_counts+=("$iterations")
    done
fi
total_cases=0
for count in "${case_counts[@]}"; do
    if [[ ! $count =~ ^[1-9][0-9]*$ ]]; then
        printf 'ERROR: invalid per-chain case count: %s\n' "$count" >&2
        exit 2
    fi
    total_cases=$((total_cases + count))
done

if [[ ! -x $scala_cli ]] && command -v scala-cli >/dev/null 2>&1; then
    scala_cli=$(command -v scala-cli)
fi
required=(
    "$scala_cli"
    "$scalepnr"
    "$nextpnr"
    "$chipdb"
    "$db_dir/tilegrid.json"
    "$db_dir/tileconn.json"
    "$db_dir/package_pins.csv"
    "$pnr_tests_dir/tests/pipeline/TestPipeline.scala"
    "$script_dir/random_pipeline.scala"
    "$script_dir/check_chain_length.py"
    "$script_dir/test.tcl"
    "$helper"
)
for path in "${required[@]}"; do
    if [[ ! -e $path ]]; then
        printf 'ERROR: required comparison input is missing: %s\n' "$path" >&2
        printf 'Run %s first.\n' "$script_dir/.prepare.sh" >&2
        exit 2
    fi
done

if ! command -v yosys >/dev/null 2>&1; then
    printf 'ERROR: yosys is not in PATH\n' >&2
    exit 2
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

run_stamp=$(date +%Y%m%d_%H%M%S)
run_parent=${SCALEPNR_COMPARE_RUN_PARENT:-$script_dir/nextpnr_runs}
mkdir -p "$run_parent"
run_parent=$(cd -- "$run_parent" && pwd)
run_root=$run_parent/size_${size}_seed_${base_seed}_${run_stamp}_${BASHPID}
summary=$run_root/summary.tsv
mkdir -p "$run_root"
printf 'chain_cap\titeration\tseed\tmeasured_chain\tsv_sha256\tcells\tnets\tports\tcell_types\torder\tscalepnr_status\tscalepnr_ms\tscalepnr_maxrss_kb\tscalepnr_unrouted\tnextpnr_status\tnextpnr_ms\tnextpnr_maxrss_kb\tnextpnr_unrouted\n' >"$summary"

run_shared_stage() {
    local name=$1
    local log=$2
    shift 2
    printf '  %-12s' "$name"
    set +e
    timeout --kill-after=15s "${shared_timeout}s" "$@" >"$log" 2>&1
    local status=$?
    set -e
    if ((status != 0)); then
        printf ' FAIL (%d)\n' "$status"
        tail -n 100 "$log" >&2 || true
        return "$status"
    fi
    printf ' OK\n'
}

run_timed_pnr() {
    local tool=$1
    local log=$2
    local metrics=$3
    local expect_clock=$4
    shift 4
    printf '  %-12s' "$tool"
    local start_ns end_ns exit_code timed_timeout
    timed_timeout=$pnr_timeout
    if [[ $tool == scalepnr ]]; then
        timed_timeout=$scalepnr_timeout
    fi
    start_ns=$(date +%s%N)
    set +e
    # Keep the command in timeout's managed process group so a hung PnR child
    # cannot survive after /usr/bin/time receives the timeout signal.
    if ((pnr_memory_kb == 0)); then
        timeout --kill-after=15s "${timed_timeout}s" \
            /usr/bin/time -f 'maxrss_kb=%M' -o "$metrics" \
            "$@" >"$log" 2>&1
    else
        timeout --kill-after=15s "${timed_timeout}s" \
            /usr/bin/time -f 'maxrss_kb=%M' -o "$metrics" \
            bash -c 'ulimit -v "$1"; shift; exec "$@"' \
            _ "$pnr_memory_kb" "$@" >"$log" 2>&1
    fi
    exit_code=$?
    set -e
    end_ns=$(date +%s%N)
    timed_ms=$(((end_ns - start_ns) / 1000000))
    timed_rss=$(sed -n 's/^maxrss_kb=//p' "$metrics" 2>/dev/null | tail -1)
    timed_rss=${timed_rss:--}
    timed_unrouted=-

    if ((exit_code == 124 || exit_code == 137)); then
        timed_status=TIMEOUT
    elif ((exit_code != 0)); then
        timed_status="FAIL_${exit_code}"
    else
        set +e
        if [[ $tool == scalepnr ]]; then
            verify_output=$(python3 "$helper" verify-scalepnr "$log" $expect_clock 2>>"$log")
        else
            verify_output=$(python3 "$helper" verify-nextpnr "$log" 2>>"$log")
        fi
        verify_status=$?
        set -e
        if ((verify_status == 0)); then
            timed_status=PASS
            timed_unrouted=$verify_output
        else
            timed_status=INVALID_ROUTE
        fi
    fi
    printf ' %-13s %8.3fs, unrouted=%s\n' \
        "$timed_status" "$(awk -v ms="$timed_ms" 'BEGIN { print ms / 1000.0 }')" \
        "$timed_unrouted"
}

printf 'Paired PnR comparison: size=%d, cases/cap=%s, caps=%s, seed=%d, nextpnr_timeout=%ds, scalepnr_timeout=%ds, route_stage_timeout=%ds, cells=%d..%s\n' \
    "$size" "${case_counts[*]}" "$chain_lengths" "$base_seed" "$pnr_timeout" \
    "$scalepnr_timeout" "$route_stage_timeout" \
    "$min_cells" "$([[ $max_cells == 0 ]] && printf unbounded || printf %d "$max_cells")"
printf 'Results: %s\n' "$run_root"
printf 'Tool versions:\n'
printf '  scalepnr: %s\n' "$(git -C "$repo_dir" rev-parse --short HEAD)"
printf '  nextpnr:  %s\n' "$($nextpnr --version 2>&1 | tr -d '"')"

failures=0
case_index=0
chain_index=0
stop_after_scalepnr_timeout=0
for chain_cap in "${chain_caps[@]}"; do
    cap_iterations=${case_counts[$chain_index]}
    chain_index=$((chain_index + 1))
    iteration=0
    candidate=0
    generation_size=$size
    while ((iteration < cap_iterations)); do
        candidate=$((candidate + 1))
        if ((candidate > max_candidates)); then
            printf 'ERROR: chain cap %d produced only %d designs with at least %d cells after %d candidates.\n' \
                "$chain_cap" "$iteration" "$min_cells" "$max_candidates" >&2
            exit 1
        fi
        # Give every chain-cap campaign a disjoint candidate seed range. This
        # prevents repeated RTL when two caps admit the same node families.
        seed=$(((base_seed + (chain_index - 1) * max_candidates + candidate - 1) % 2147483647))
        work_dir=$run_root/chain_${chain_cap}/candidate_$(printf '%03d' "$candidate")_seed_${seed}
        generated_dir=$work_dir/generated
        mkdir -p "$generated_dir"
        printf '\n[chain=%d candidate=%d accepted=%02d/%02d seed=%d stages=%d]\n' \
            "$chain_cap" "$candidate" "$iteration" "$cap_iterations" "$seed" \
            "$generation_size"

        if ! run_shared_stage generate "$work_dir/generate.log" \
                flock "$script_dir/.tools/random_generator.lock" \
                "$scala_cli" run "${pnr_sources[@]}" \
                --server=false "${scala_jvm_args[@]}" \
                --main-class ScalepnrRandomPipeline -- \
                "$generated_dir" random_design "$part" "$db_dir" "$complexity" \
                "$generation_size" "$max_width" "$chain_cap" "$seed"; then
            printf '  generate      REJECT seed=%d; trying the next candidate\n' "$seed"
            continue
        fi

        sv_file=$generated_dir/TestPipeline.sv
        raw_xdc=$generated_dir/random_design.xdc
        constraints=$generated_dir/scalepnr_constraints.tcl
        python3 - "$raw_xdc" "$constraints" <<'PY'
from pathlib import Path
import re
import sys

source = Path(sys.argv[1])
target = Path(sys.argv[2])
pattern = re.compile(r"\[get_ports (.+)\]\s*$")
lines = []
for raw_line in source.read_text().splitlines():
    match = pattern.search(raw_line)
    if match:
        raw_line = raw_line[:match.start()] + "[get_ports {" + match.group(1) + "}]"
    lines.append(raw_line)
target.write_text("\n".join(lines) + "\n")
PY

        if ! run_shared_stage synthesis "$work_dir/yosys.log" \
                bash -c 'cd "$1" && exec yosys -p "read_verilog -sv TestPipeline.sv; clkbufmap -inpad IBUFG *clock*; synth_xilinx -nobram -nolutram -nodsp -nocarry -flatten -arch xc7 -top TestPipeline; delete t:\$scopeinfo; write_json test.json"' \
                _ "$generated_dir"; then
            printf '  synthesis     REJECT seed=%d; trying the next candidate\n' "$seed"
            continue
        fi

        design_json=$generated_dir/test.json
        IFS=$'\t' read -r cells nets ports sequential cell_types < <(
            python3 "$helper" design-stats "$design_json" --top "$top"
        )

        # SIZE remains an initial estimate. Scale subsequent candidates from
        # actual Yosys output until they fit the requested synthesized range.
        if ((min_cells != 0 && cells < min_cells)) ||
           ((max_cells != 0 && cells > max_cells)); then
            if ((cells < min_cells)); then
                target_cells=$min_cells
                scale_margin=103
            else
                target_cells=$(((min_cells + max_cells) / 2))
                scale_margin=100
            fi
            scaled_size=$(((generation_size * target_cells + cells - 1) / cells))
            scaled_size=$(((scaled_size * scale_margin + 99) / 100))
            if ((cells < min_cells && scaled_size <= generation_size)); then
                scaled_size=$((generation_size + 1))
            elif ((max_cells != 0 && cells > max_cells && scaled_size >= generation_size)); then
                scaled_size=$((generation_size - 1))
            fi
            if ((scaled_size < 1)); then
                scaled_size=1
            fi
            printf '  size-target  cells=%d required=%d..%s; stages %d -> %d\n' \
                "$cells" "$min_cells" \
                "$([[ $max_cells == 0 ]] && printf unbounded || printf %d "$max_cells")" \
                "$generation_size" "$scaled_size"
            generation_size=$scaled_size
        fi

        chain_result=$work_dir/chain_length.txt
        if ! run_shared_stage chain-check "$work_dir/chain_check.log" \
                python3 "$script_dir/check_chain_length.py" "$design_json" \
                --top "$top" --maximum "$chain_cap" --result "$chain_result"; then
            printf '  chain-check  REJECT seed=%d; trying the next candidate\n' "$seed"
            continue
        fi
        measured_chain=$(<"$chain_result")

        if ((cells < min_cells)); then
            printf '  size-check   REJECT cells=%d, required>=%d\n' "$cells" "$min_cells"
            continue
        fi
        if ((max_cells != 0 && cells > max_cells)); then
            printf '  size-check   REJECT cells=%d, required<=%d\n' "$cells" "$max_cells"
            continue
        fi
        iteration=$((iteration + 1))
        case_index=$((case_index + 1))
        printf '  size-check   ACCEPT cells=%d, accepted=%02d/%02d\n' \
            "$cells" "$iteration" "$cap_iterations"
        if ((sequential != 0)); then
            python3 - "$constraints" "$freq_mhz" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
period_ns = 1000.0 / float(sys.argv[2])
path.write_text(
    f"create_clock -name clock -period {period_ns:.6f} [get_ports {{clock}}]\n"
    + path.read_text()
)
PY
            expect_clock=--expect-clock
        else
            expect_clock=
        fi

        scalepnr_log=$work_dir/scalepnr.log
        nextpnr_log=$work_dir/nextpnr.log
        if ((case_index % 2 == 1)); then
            order=scalepnr-first
            tools=(scalepnr nextpnr)
        else
            order=nextpnr-first
            tools=(nextpnr scalepnr)
        fi

        scalepnr_status=NOT_RUN
        scalepnr_ms=0
        scalepnr_rss=-
        scalepnr_unrouted=-
        nextpnr_status=NOT_RUN
        nextpnr_ms=0
        nextpnr_rss=-
        nextpnr_unrouted=-

        for tool in "${tools[@]}"; do
            if [[ $tool == scalepnr ]]; then
                run_timed_pnr scalepnr "$scalepnr_log" "$work_dir/scalepnr.time" \
                    "$expect_clock" \
                    env \
                    SCALEPNR_TEST_JSON="$design_json" \
                    SCALEPNR_TEST_TOP="$top" \
                    SCALEPNR_TEST_CONSTRAINTS="$constraints" \
                    SCALEPNR_QUIET_ROUTES=1 \
                    SCALEPNR_SKIP_WRITE_DESIGN=1 \
                    SCALEPNR_ROUTE_HEARTBEAT=0 \
                    SCALEPNR_ROUTE_STAGE_TIMEOUT="$route_stage_timeout" \
                    SCALEPNR_FAILURE_ARTIFACT_DIR="$work_dir" \
                    "$scalepnr" "$script_dir/test.tcl"
                scalepnr_status=$timed_status
                scalepnr_ms=$timed_ms
                scalepnr_rss=$timed_rss
                scalepnr_unrouted=$timed_unrouted
                if [[ -n ${SCALEPNR_COMPARE_STOP_ON_SCALEPNR_TIMEOUT:-} &&
                      ( $scalepnr_status == TIMEOUT ||
                        $scalepnr_status == FAIL_139 ) ]]; then
                    stop_after_scalepnr_timeout=1
                    break
                fi
            else
                run_timed_pnr nextpnr "$nextpnr_log" "$work_dir/nextpnr.time" "" \
                    "$nextpnr" \
                    --chipdb "$chipdb" \
                    --xdc "$raw_xdc" \
                    --json "$design_json" \
                    --freq "$freq_mhz" \
                    --placer-heap-cell-placement-timeout 0 \
                    --timing-allow-fail
                nextpnr_status=$timed_status
                nextpnr_ms=$timed_ms
                nextpnr_rss=$timed_rss
                nextpnr_unrouted=$timed_unrouted
            fi
        done

        sv_hash=$(sha256sum "$sv_file" | awk '{print $1}')
        printf '%d\t%d\t%d\t%s\t%s\t%d\t%d\t%d\t%s\t%s\t%s\t%d\t%s\t%s\t%s\t%d\t%s\t%s\n' \
            "$chain_cap" "$iteration" "$seed" "$measured_chain" "$sv_hash" \
            "$cells" "$nets" "$ports" "$cell_types" "$order" \
            "$scalepnr_status" "$scalepnr_ms" "$scalepnr_rss" "$scalepnr_unrouted" \
            "$nextpnr_status" "$nextpnr_ms" "$nextpnr_rss" "$nextpnr_unrouted" \
            >>"$summary"

        if [[ $scalepnr_status != PASS || $nextpnr_status != PASS ]]; then
            failures=$((failures + 1))
            printf '  FAILURE artifacts: %s\n' "$work_dir" >&2
            if [[ -n ${SCALEPNR_COMPARE_FAIL_FAST:-} ]]; then
                exit 1
            fi
        fi
        if ((stop_after_scalepnr_timeout)); then
            break
        fi
    done
    if ((stop_after_scalepnr_timeout)); then
        break
    fi
done

printf '\nSummary: %s\n' "$summary"
awk -F '\t' 'NR > 1 {
    key=$1
    count[key]++
    if ($11 == "PASS") { spass[key]++; stime[key]+=$12 }
    if ($15 == "PASS") { npass[key]++; ntime[key]+=$16 }
}
END {
    print "chain  cases  scalepnr_pass  scalepnr_mean_s  nextpnr_pass  nextpnr_mean_s"
    for (key in count) {
        sm=(spass[key] ? stime[key]/spass[key]/1000 : 0)
        nm=(npass[key] ? ntime[key]/npass[key]/1000 : 0)
        printf "%5s  %5d  %13d  %15.3f  %12d  %14.3f\n", key, count[key], spass[key], sm, npass[key], nm
    }
}' "$summary"

if ((failures != 0)); then
    printf 'FAIL: %d paired cases had a timeout, process failure, or nonzero unrouted result.\n' \
        "$failures" >&2
    exit 1
fi
printf 'PASS: all %d paired cases completed with zero unrouted nets.\n' \
    "$total_cases"
