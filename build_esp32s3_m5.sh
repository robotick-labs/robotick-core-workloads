#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENGINE_ROOT="${ROBOTICK_ENGINE_SOURCE_DIR:-${ROOT_DIR}/../robotick-engine}"
IMAGE="${ESP32_IDF_IMAGE:-espressif/idf:release-v5.4}"

if ! command -v docker >/dev/null 2>&1; then
  echo "[build_esp32s3_m5] ❌ docker command not found. Please install Docker and try again." >&2
  exit 1
fi

if [ ! -d "${ENGINE_ROOT}" ]; then
  echo "[build_esp32s3_m5] ❌ Unable to locate robotick-engine at '${ENGINE_ROOT}'. Set ROBOTICK_ENGINE_SOURCE_DIR to the engine checkout." >&2
  exit 1
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[build_esp32s3_m5] Pulling Docker image ${IMAGE}..."
  docker pull "${IMAGE}"
fi

echo "[build_esp32s3_m5] Starting ESP32-S3+M5 build using ${IMAGE}..."
docker run --rm \
  --user root \
  -v "${ROOT_DIR}:/workspace/robotick-core-workloads" \
  -v "${ENGINE_ROOT}:/workspace/robotick-engine" \
  -w /workspace/robotick-core-workloads \
  "${IMAGE}" \
bash -c '
set -Eeuo pipefail
export TERM=xterm-256color
export ROBOTICK_PLATFORM_ESP32S3_M5=1
export IDF_EXTRA_CMAKE_ARGS='-DROBOTICK_PLATFORM_ESP32S3=ON -DROBOTICK_PLATFORM_ESP32S3_M5=ON'
set -x

cd /workspace/robotick-core-workloads/tools/esp32-compile-check

if ! command -v ninja >/dev/null 2>&1; then
  DEBIAN_FRONTEND=noninteractive apt-get update -yq
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ninja-build ccache
  apt-get clean
  rm -rf /var/lib/apt/lists/*
fi

bash /workspace/robotick-core-workloads/tools/make_esp32_symlinks.sh

./1_idf_clean.sh
./2_idf_build.sh
'

echo "[build_esp32s3_m5] ✅ ESP32-S3+M5 build finished successfully."
