#!/usr/bin/env bash
set -uo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$script_dir/../.."
if ! python3 "$script_dir/cleanup_logs.py" --apply --tmp-root "${TMPDIR:-/tmp}"; then
    exit 2
fi
export SCALEPNR_PNR_RUN_CELLS="${SCALEPNR_PNR_RUN_CELLS:-5000}"
binary="${SCALEPNR_BINARY:-$PWD/build-release/scalepnr}"
process_timeout="${SCALEPNR_PROCESS_TIMEOUT:-19200}"
min_available_kib="${SCALEPNR_MIN_AVAILABLE_KB:-204800}"
virtual_limit_kib="${SCALEPNR_RUN_MEMORY_KB:-6291456}"
min_disk_kib="${SCALEPNR_MIN_DISK_KB:-1048576}"
max_log_bytes="${SCALEPNR_MAX_LOG_BYTES:-268435456}"
for limit in "$min_available_kib" "$virtual_limit_kib" "$process_timeout" "$min_disk_kib" "$max_log_bytes"; do
    if [[ ! "$limit" =~ ^[1-9][0-9]*$ ]]; then
        echo "resource limits and timeout must be positive integers" >&2
        exit 2
    fi
done
if (( max_log_bytes < 1024 )); then
    echo "SCALEPNR_MAX_LOG_BYTES must be at least 1024" >&2
    exit 2
fi
ulimit -v "$virtual_limit_kib" || exit 2
ulimit -c 0
candidate="$PWD/tests/prjxray/nextpnr_runs/size_132_seed_5000_20260822_112830_118638/chain_10/candidate_001_seed_5000"
run="${SCALEPNR_DIAGNOSTIC_RUN_DIR:-$PWD/tests/prjxray/runs/50k_$(date +%Y%m%d_%H%M%S)_${BASHPID}}"
mkdir -p "$run"
log="$run/run.log"
# Write straight to a regular file: no pipe, compressor, spool, or archive.
set -o noclobber
exec 3> "$log" || exit 2
export TCL_LIBRARY="$PWD/libs/tcl8.6.14/library"
export SCALEPNR_TEST_JSON="${SCALEPNR_TEST_JSON:-$candidate/generated/test.json}" SCALEPNR_TEST_TOP="${SCALEPNR_TEST_TOP:-TestPipeline}"
export SCALEPNR_TEST_CONSTRAINTS="${SCALEPNR_TEST_CONSTRAINTS:-$candidate/generated/scalepnr_constraints.tcl}"
export SCALEPNR_PLACEMENT_DB="${SCALEPNR_DIAGNOSTIC_PLACEMENT_DB:-$run/placement.db}"
if [[ -n "${SCALEPNR_PNR_RUN_CELLS:-}" ]]; then
    unset SCALEPNR_PLACEMENT_DB
fi
export SCALEPNR_DESIGN_DB="${SCALEPNR_DIAGNOSTIC_DESIGN_DB:-$run/design_state.db}" SCALEPNR_FAILURE_ARTIFACT_DIR="$run"
export SCALEPNR_QUIET_ROUTES=1 SCALEPNR_SKIP_READBACK=1 SCALEPNR_ROUTE_HEARTBEAT=1
export SCALEPNR_PROFILE_PLACEMENT=1
export SCALEPNR_ROUTE_PACKING_TRACE=0
export SCALEPNR_ROUTE_STAGE_TIMEOUT="${SCALEPNR_ROUTE_STAGE_TIMEOUT:-2400}" SCALEPNR_ROUTE_CLOCK_TIMEOUT="${SCALEPNR_ROUTE_CLOCK_TIMEOUT:-2400}"
export SCALEPNR_FREE_SEARCH_STATES="${SCALEPNR_FREE_SEARCH_STATES:-65536}"
export SCALEPNR_ROUTE_PROGRESS_WINDOW=60 SCALEPNR_ROUTE_STAGNANT_WINDOWS="${SCALEPNR_ROUTE_STAGNANT_WINDOWS:-10}"
unset SCALEPNR_CLOCK_ONLY SCALEPNR_SKIP_WRITE_DESIGN
printf 'RUN_START time=%s seed=5000 chain_limit=10 original_cells=51749 sparse_load_max_percent=50 stage_timeout_s=%s process_timeout_s=%s virtual_limit_kib=%s min_available_kib=%s\nplacement_db=%s run_cells=%s\n' "$(date -Is)" "$SCALEPNR_ROUTE_STAGE_TIMEOUT" "$process_timeout" "$virtual_limit_kib" "$min_available_kib" "${SCALEPNR_PLACEMENT_DB:-none}" "${SCALEPNR_PNR_RUN_CELLS:-standalone}" >&3
printf 'free_search_states=%s\n' "$SCALEPNR_FREE_SEARCH_STATES" >&3
git rev-parse HEAD >&3
git status --short >&3
git diff --stat >&3
sha256sum "$binary" >&3
debugger=()
if [[ -n "${SCALEPNR_MEMORY_GDB:-}" ]]; then
    debugger=(gdb -q -batch -x "$SCALEPNR_MEMORY_GDB" --args)
fi
command=(stdbuf -oL -e0 "$binary" tests/prjxray/test.tcl)
if [[ -n "${SCALEPNR_HEAP_SAMPLE:-}" ]]; then
    command=(env "LD_PRELOAD=$SCALEPNR_HEAP_SAMPLE" "MALLOC_CONF=prof:true,lg_prof_sample:20,prof_prefix:$run/heap" "${SCALEPNR_PROFILE_BINARY:-$binary}" tests/prjxray/test.tcl)
fi
if [[ -n "${SCALEPNR_CPU_PROFILE:-}" ]]; then
    command=(env LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libprofiler.so.0 "CPUPROFILE=$SCALEPNR_CPU_PROFILE" CPUPROFILESIGNAL=12 CPUPROFILE_FREQUENCY=99 "$binary" tests/prjxray/test.tcl)
fi
setsid /usr/bin/time -v timeout --foreground --kill-after=10s "${process_timeout}s" \
    "${debugger[@]}" "${command[@]}" >&3 2>&1 &
job=$!
echo "process_group=$job log=$log"
tick=0
while kill -0 "$job" 2>/dev/null; do
    available=$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)
    if (( available < min_available_kib )); then
        echo "MEMORY_GUARD time=$(date -Is) available_kib=$available min_available_kib=$min_available_kib stopping run" >&3
        ps -eo pid,pgid,rss,vsz,comm | awk -v group="$job" \
            '$2 == group && $5 == "scalepnr" {printf "MEMORY_GUARD_PROCESS pid=%s rss_kib=%s virtual_kib=%s\n",$1,$3,$4}' >&3
        kill -TERM -- "-$job" 2>/dev/null || true
        sleep 1
        kill -KILL -- "-$job" 2>/dev/null || true
        break
    fi
    if (( tick % 4 == 0 )); then
        disk_available=$(df -Pk "$run" | awk 'NR == 2 {print $4}')
        if (( disk_available < min_disk_kib )); then
            echo "DISK_GUARD time=$(date -Is) available_kib=$disk_available stopping run" >&3
            kill -TERM -- "-$job" 2>/dev/null || true
            sleep 1
            kill -KILL -- "-$job" 2>/dev/null || true
            break
        fi
    fi
    if (( tick % 120 == 0 )); then
        ps -eo pid,pgid,rss,vsz,comm | awk -v group="$job" -v stamp="$(date -Is)" \
            '$2 == group && $5 == "scalepnr" {printf "\nMEMORY_SAMPLE time=%s pid=%s rss_kib=%s virtual_kib=%s\n",stamp,$1,$3,$4}' >&3
    fi
    tick=$((tick+1))
    sleep 0.25
done
wait "$job"
status=$?
echo "RUN_END time=$(date -Is) exit=$status" >&3
exec 3>&-
python3 "$script_dir/cleanup_logs.py" --apply --tmp-root "${TMPDIR:-/tmp}" \
    --truncate-log "$log" --max-log-bytes "$max_log_bytes"
echo "exit=$status log=$log"
exit "$status"

