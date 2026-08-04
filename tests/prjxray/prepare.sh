#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PRJXRAY_DIR="${ROOT_DIR}/prjxray"
PRJXRAY_DB_DIR="${ROOT_DIR}/prjxray-db"
PNR_TESTS_DIR="${ROOT_DIR}/pnr_tests"
TOOLS_DIR="${ROOT_DIR}/.tools"
SCALA_CLI="${SCALA_CLI:-${TOOLS_DIR}/scala-cli}"
SCALA_CLI_VERSION="${SCALA_CLI_VERSION:-1.15.0}"
DB_FAMILY="artix7"
DB_PART="xc7a100t"
DB_PACKAGE="xc7a100tfgg676-1"
PRJXRAY_CC="${PRJXRAY_CC:-/usr/bin/gcc}"
PRJXRAY_CXX="${PRJXRAY_CXX:-/usr/bin/g++}"
PRJXRAY_CXXFLAGS="${PRJXRAY_CXXFLAGS:--Wno-error=free-nonheap-object}"

if ! command -v git >/dev/null 2>&1; then
    echo "git is required" >&2
    exit 1
fi

if ! command -v cmake >/dev/null 2>&1; then
    echo "cmake is required to build prjxray" >&2
    exit 1
fi

if ! command -v make >/dev/null 2>&1; then
    echo "make is required to build prjxray" >&2
    exit 1
fi

if ! command -v java >/dev/null 2>&1; then
    for java_home in "${HOME}"/Xilinx/Vivado/*/tps/lnx64/jre* /opt/Xilinx/Vivado/*/tps/lnx64/jre*; do
        if [ -x "${java_home}/bin/java" ]; then
            export JAVA_HOME="${java_home}"
            export PATH="${JAVA_HOME}/bin:${PATH}"
            break
        fi
    done
fi

if [ ! -d "${PNR_TESTS_DIR}/.git" ]; then
    git clone https://github.com/mirekez/pnr_tests.git "${PNR_TESTS_DIR}"
fi

if [ ! -x "${SCALA_CLI}" ]; then
    if command -v scala-cli >/dev/null 2>&1; then
        SCALA_CLI="$(command -v scala-cli)"
    else
        if ! command -v curl >/dev/null 2>&1 || ! command -v gzip >/dev/null 2>&1; then
            echo "curl and gzip are required to install scala-cli" >&2
            exit 1
        fi
        mkdir -p "${TOOLS_DIR}"
        echo "Installing scala-cli into ${SCALA_CLI}"
        curl -fL "https://github.com/VirtusLab/scala-cli/releases/download/v${SCALA_CLI_VERSION}/scala-cli-x86_64-pc-linux.gz" \
            | gzip -dc > "${SCALA_CLI}"
        chmod +x "${SCALA_CLI}"
    fi
fi

PNR_TESTS_SOURCES=(
    "${PNR_TESTS_DIR}/queue/NodeQueue.scala"
    "${PNR_TESTS_DIR}/mux/NodeMux.scala"
    "${PNR_TESTS_DIR}/mux/NodeDemux.scala"
    "${PNR_TESTS_DIR}/math/NodeMul.scala"
    "${PNR_TESTS_DIR}/math/NodeDiv.scala"
    "${PNR_TESTS_DIR}/decode/NodeMap.scala"
    "${PNR_TESTS_DIR}/memory/NodeMemory.scala"
    "${PNR_TESTS_DIR}/common/AttributeAnnotation.scala"
    "${PNR_TESTS_DIR}/tests/Crossbar.scala"
    "${PNR_TESTS_DIR}/tests/NodeFabric.scala"
    "${PNR_TESTS_DIR}/tests/XDCGen.scala"
    "${PNR_TESTS_DIR}/tests/pipeline/TestPipeline.scala"
    "${ROOT_DIR}/random_pipeline.scala"
)
echo "Building the scalepnr pnr_tests generator"
SCALA_JVM_ARGS=()
if command -v java >/dev/null 2>&1; then
    SCALA_JVM_ARGS=(--jvm system)
fi
"${SCALA_CLI}" compile --server=false "${SCALA_JVM_ARGS[@]}" "${PNR_TESTS_SOURCES[@]}"

if [ ! -d "${PRJXRAY_DIR}/.git" ]; then
    git clone https://github.com/f4pga/prjxray.git "${PRJXRAY_DIR}"
fi

if [ -f "${PRJXRAY_DIR}/third_party/yosys/.git" ] && ! git -C "${PRJXRAY_DIR}/third_party/yosys" rev-parse --verify HEAD >/dev/null 2>&1; then
    echo "Recovering interrupted third_party/yosys submodule checkout"
    rm -rf "${PRJXRAY_DIR}/third_party/yosys" "${PRJXRAY_DIR}/.git/modules/third_party/yosys"
fi

git -C "${PRJXRAY_DIR}" submodule update --init --recursive

PRJXRAY_MMAP_HEADER="${PRJXRAY_DIR}/lib/include/prjxray/memory_mapped_file.h"
if [ -f "${PRJXRAY_MMAP_HEADER}" ] && ! grep -q "#include <cstdint>" "${PRJXRAY_MMAP_HEADER}"; then
    python3 - "${PRJXRAY_MMAP_HEADER}" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text()
text = text.replace("#include <memory>\n", "#include <cstdint>\n#include <memory>\n", 1)
path.write_text(text)
PY
fi

if [ -f "${PRJXRAY_DIR}/build/CMakeCache.txt" ] && grep -q "w64-mingw32" "${PRJXRAY_DIR}/build/CMakeCache.txt"; then
    echo "Removing prjxray build cache configured with MinGW cross compiler"
    rm -rf "${PRJXRAY_DIR}/build"
fi
if [ -f "${PRJXRAY_DIR}/build/CMakeCache.txt" ] && ! grep -q "Wno-error=free-nonheap-object" "${PRJXRAY_DIR}/build/CMakeCache.txt"; then
    echo "Removing prjxray build cache configured without GCC 15 warning override"
    rm -rf "${PRJXRAY_DIR}/build"
fi
env CC="${PRJXRAY_CC}" CXX="${PRJXRAY_CXX}" CXXFLAGS="${PRJXRAY_CXXFLAGS}" make -C "${PRJXRAY_DIR}" build

if [ ! -d "${PRJXRAY_DB_DIR}/.git" ]; then
    git clone --filter=blob:none --no-checkout https://github.com/SymbiFlow/prjxray-db.git "${PRJXRAY_DB_DIR}"
    git -C "${PRJXRAY_DB_DIR}" sparse-checkout init --cone
    git -C "${PRJXRAY_DB_DIR}" sparse-checkout set \
        "${DB_FAMILY}/${DB_PART}" \
        "${DB_FAMILY}/mapping" \
        "${DB_FAMILY}/${DB_PACKAGE}"
    git -C "${PRJXRAY_DB_DIR}" checkout
else
    git -C "${PRJXRAY_DB_DIR}" sparse-checkout add \
        "${DB_FAMILY}/mapping" \
        "${DB_FAMILY}/${DB_PACKAGE}"
fi

DB_SOURCE="${PRJXRAY_DB_DIR}/${DB_FAMILY}/${DB_PART}"
if [ ! -d "${DB_SOURCE}" ]; then
    echo "Expected database folder not found: ${DB_SOURCE}" >&2
    exit 1
fi

DB_DIR="${ROOT_DIR}/db"
rm -rf "${DB_DIR}"
mkdir -p "${DB_DIR}"

ln -sfn "${DB_SOURCE}/tilegrid.json" "${DB_DIR}/tilegrid.json"
ln -sfn "${DB_SOURCE}/tileconn.json" "${DB_DIR}/tileconn.json"
ln -sfn "${DB_SOURCE}/node_wires.json" "${DB_DIR}/node_wires.json"

for spec_path in "${PRJXRAY_DB_DIR}/${DB_FAMILY}"/tile_type_*.json; do
    [ -e "${spec_path}" ] || continue
    ln -sfn "${spec_path}" "${DB_DIR}/$(basename "${spec_path}")"
done

PACKAGE_PINS_SOURCE="${DB_SOURCE}/package_pins.csv"
if [ ! -f "${PACKAGE_PINS_SOURCE}" ]; then
    PACKAGE_PINS_SOURCE="${PRJXRAY_DB_DIR}/${DB_FAMILY}/${DB_PACKAGE}/package_pins.csv"
fi
if [ ! -f "${PACKAGE_PINS_SOURCE}" ]; then
    PACKAGE_PINS_SOURCE="${ROOT_DIR}/../../xc7a100t/package_pins.csv"
fi
if [ ! -f "${PACKAGE_PINS_SOURCE}" ]; then
    echo "Expected package_pins.csv not found in prjxray-db or ../../xc7a100t/package_pins.csv" >&2
    exit 1
fi
ln -sfn "${PACKAGE_PINS_SOURCE}" "${DB_DIR}/package_pins.csv"

DB_PACKAGE_DIR="${PRJXRAY_DB_DIR}/${DB_FAMILY}/${DB_PACKAGE}"
mkdir -p "${DB_PACKAGE_DIR}"
if [ "$(readlink -f "${PACKAGE_PINS_SOURCE}")" != "$(readlink -f "${DB_PACKAGE_DIR}/package_pins.csv" 2>/dev/null || true)" ]; then
    ln -sfn "${PACKAGE_PINS_SOURCE}" "${DB_PACKAGE_DIR}/package_pins.csv"
fi
if [ ! -f "${DB_PACKAGE_DIR}/part.json" ]; then
    printf '{"iobanks": {}}\n' > "${DB_PACKAGE_DIR}/part.json"
fi

FASM2BIT="${ROOT_DIR}/fasm2bit"
cat > "${FASM2BIT}" <<EOF
#!/usr/bin/env bash
set -euo pipefail

export XRAY_DATABASE_DIR="${PRJXRAY_DB_DIR}"
export XRAY_DATABASE="${DB_FAMILY}"
export XRAY_PART="${DB_PACKAGE}"
export PATH="${PRJXRAY_DIR}/build/tools:\${PATH}"
export PYTHONPATH="${ROOT_DIR}:${PRJXRAY_DIR}:\${PYTHONPATH:-}"

if [ -x "${PRJXRAY_DIR}/utils/fasm2bit.sh" ]; then
    exec "${PRJXRAY_DIR}/utils/fasm2bit.sh" "\$@"
fi

if [ -f "${PRJXRAY_DIR}/utils/fasm2bit.py" ]; then
    exec python3 "${PRJXRAY_DIR}/utils/fasm2bit.py" "\$@"
fi

if [ -x "${PRJXRAY_DIR}/minitests/roi_harness/fasm2bit.sh" ]; then
    exec "${PRJXRAY_DIR}/minitests/roi_harness/fasm2bit.sh" "\$@"
fi

echo "No fasm2bit entry point found in ${PRJXRAY_DIR}" >&2
echo "Looked for utils/fasm2bit.sh, utils/fasm2bit.py, and minitests/roi_harness/fasm2bit.sh" >&2
exit 1
EOF
chmod +x "${FASM2BIT}"

echo "Prepared prjxray at ${PRJXRAY_DIR}"
echo "Prepared ${DB_FAMILY}/${DB_PART} database at ${DB_SOURCE}"
echo "Assembled database for scalepnr test at ${DB_DIR}"
echo "Installed fasm2bit wrapper at ${FASM2BIT}"
echo "Prepared pnr_tests at ${PNR_TESTS_DIR}"
echo "Prepared scala-cli at ${SCALA_CLI}"
