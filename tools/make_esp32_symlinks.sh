#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPONENTS_DIR="${ROOT_DIR}/platforms/esp32/components"
ENGINE_ROOT="${ROBOTICK_ENGINE_SOURCE_DIR:-${ROOT_DIR}/../robotick-engine}"

if [ ! -d "${ENGINE_ROOT}" ]; then
  echo "[make_esp32_symlinks] ❌ Unable to locate robotick-engine at '${ENGINE_ROOT}'. Set ROBOTICK_ENGINE_SOURCE_DIR to the engine checkout." >&2
  exit 1
fi

link_items() {
  local target_dir="$1"
  local source_dir="$2"
  shift 2

  mkdir -p "${target_dir}"

  for item in "$@"; do
    local src="${source_dir}/${item}"
    if [ -e "${src}" ]; then
      rm -rf "${target_dir}/${item}"
      ln -snf "${src}" "${target_dir}/${item}"
    fi
  done
}

rm -rf "${COMPONENTS_DIR}/robotick-engine"
ln -snf "${ENGINE_ROOT}/platforms/esp32/components/robotick-engine" "${COMPONENTS_DIR}/robotick-engine"

link_items "${COMPONENTS_DIR}/robotick-core-workloads" "${ROOT_DIR}/cpp" \
  "CMakeWorkloads.json" \
  "include" \
  "src"

echo "[make_esp32_symlinks] ✅ Symlinked robotick-engine and core workloads into platforms/esp32/components"
