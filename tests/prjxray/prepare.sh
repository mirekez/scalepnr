#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PRJXRAY_DIR="${ROOT_DIR}/prjxray"
PRJXRAY_DB_DIR="${ROOT_DIR}/prjxray-db"
DB_FAMILY="artix7"
DB_PART="xc7a100t"
DB_PACKAGE="xc7a100tfgg676-1"
PRJXRAY_CC="${PRJXRAY_CC:-/usr/bin/gcc}"
PRJXRAY_CXX="${PRJXRAY_CXX:-/usr/bin/g++}"

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

if [ ! -d "${PRJXRAY_DIR}/.git" ]; then
    git clone https://github.com/f4pga/prjxray.git "${PRJXRAY_DIR}"
fi

if [ -f "${PRJXRAY_DIR}/third_party/yosys/.git" ] && ! git -C "${PRJXRAY_DIR}/third_party/yosys" rev-parse --verify HEAD >/dev/null 2>&1; then
    echo "Recovering interrupted third_party/yosys submodule checkout"
    rm -rf "${PRJXRAY_DIR}/third_party/yosys" "${PRJXRAY_DIR}/.git/modules/third_party/yosys"
fi

git -C "${PRJXRAY_DIR}" submodule update --init --recursive
if [ -f "${PRJXRAY_DIR}/build/CMakeCache.txt" ] && grep -q "w64-mingw32" "${PRJXRAY_DIR}/build/CMakeCache.txt"; then
    echo "Removing prjxray build cache configured with MinGW cross compiler"
    rm -rf "${PRJXRAY_DIR}/build"
fi
env CC="${PRJXRAY_CC}" CXX="${PRJXRAY_CXX}" make -C "${PRJXRAY_DIR}" build

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
    PACKAGE_PINS_SOURCE="${ROOT_DIR}/../../xc7a100t/package_pins.csv"
fi
if [ ! -f "${PACKAGE_PINS_SOURCE}" ]; then
    echo "Expected package_pins.csv not found in prjxray-db or ../../xc7a100t/package_pins.csv" >&2
    exit 1
fi
ln -sfn "${PACKAGE_PINS_SOURCE}" "${DB_DIR}/package_pins.csv"

DB_PACKAGE_DIR="${PRJXRAY_DB_DIR}/${DB_FAMILY}/${DB_PACKAGE}"
mkdir -p "${DB_PACKAGE_DIR}"
ln -sfn "${PACKAGE_PINS_SOURCE}" "${DB_PACKAGE_DIR}/package_pins.csv"
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
