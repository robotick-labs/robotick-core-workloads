#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENGINE_ROOT="${ROBOTICK_ENGINE_SOURCE_DIR:-${ROOT_DIR}/../robotick-engine}"
IMAGE="${ESP32_IDF_IMAGE:-espressif/idf:release-v5.4}"

if ! command -v docker >/dev/null 2>&1; then
  echo "[build_esp32s3] ❌ docker command not found. Please install Docker and try again." >&2
  exit 1
fi

if [ ! -d "${ENGINE_ROOT}" ]; then
  echo "[build_esp32s3] ❌ Unable to locate robotick-engine at '${ENGINE_ROOT}'. Set ROBOTICK_ENGINE_SOURCE_DIR to the engine checkout." >&2
  exit 1
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[build_esp32s3] Pulling Docker image ${IMAGE}..."
  docker pull "${IMAGE}"
fi

echo "[build_esp32s3] Starting ESP32-S3 build using ${IMAGE}..."
docker run --rm \
  --user root \
  -v "${ROOT_DIR}:/workspace/robotick-core-workloads" \
  -v "${ENGINE_ROOT}:/workspace/robotick-engine" \
  -w /workspace/robotick-core-workloads \
  "${IMAGE}" \
bash -c '
              set -Eeuo pipefail
              export TERM=xterm-256color
              unset ROBOTICK_PLATFORM_ESP32S3_M5
              export IDF_EXTRA_CMAKE_ARGS="-DROBOTICK_PLATFORM_ESP32S3=ON -DROBOTICK_PLATFORM_ESP32S3_M5=OFF"
set -x

cd /workspace/robotick-core-workloads/tools/esp32-compile-check

if ! command -v ninja >/dev/null 2>&1; then
  DEBIAN_FRONTEND=noninteractive apt-get update -yq
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ninja-build ccache
  apt-get clean
  rm -rf /var/lib/apt/lists/*
fi

bash /workspace/robotick-core-workloads/tools/make_esp32_symlinks.sh

COMPONENTS_DIR="/workspace/robotick-core-workloads/tools/esp32-compile-check/components"
M5_UNIFIED_DIR="${COMPONENTS_DIR}/M5Unified"
M5_GFX_DIR="${COMPONENTS_DIR}/M5GFX"
BACKUP_SUFFIX=".disabled-backup"

restore_m5_components() {
  for dir in "M5Unified" "M5GFX"; do
    local backup="${COMPONENTS_DIR}/.${dir}${BACKUP_SUFFIX}"
    local target="${COMPONENTS_DIR}/${dir}"
    if [ -d "${backup}" ] && [ ! -d "${target}" ]; then
      mv "${backup}" "${target}"
    fi
  done
}

trap restore_m5_components EXIT

for dir in "M5Unified" "M5GFX"; do
  if [ -d "${COMPONENTS_DIR}/${dir}" ]; then
    mv "${COMPONENTS_DIR}/${dir}" "${COMPONENTS_DIR}/.${dir}${BACKUP_SUFFIX}"
  fi
done

./1_idf_clean.sh
./2_idf_build.sh
'

echo "[build_esp32s3] ✅ ESP32-S3 build finished successfully."
