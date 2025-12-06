#!/usr/bin/env bash
set -euo pipefail

echo "Installing robotick_core_workloads_tests dependencies..."

# Silence apt UI and change list handling to non-interactive
export DEBIAN_FRONTEND=noninteractive
export APT_LISTCHANGES_FRONTEND=none

# ----------------------------------------
# Core development tools and libraries
# ----------------------------------------

apt-get update -yq
apt-get install -y --no-install-recommends \
  git cmake build-essential pkg-config \
  libopencv-dev \
  libsdl2-dev libsdl2-ttf-dev libsdl2-gfx-dev \
  libssl-dev libcurl4-openssl-dev \
  python3-dev \
  libyaml-cpp-dev \
  libkissfft-dev

# Clean up to reduce image size if running in Docker or CI
apt-get clean
rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# Download Whisper model (ggml-base.en.bin) for tests
# ---------------------------------------------------------

WHISPER_MODEL_URL="https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin"
WHISPER_MODEL_DIR="cpp/tests/data/models/whisper"
WHISPER_MODEL_PATH="${WHISPER_MODEL_DIR}/ggml-base.en.bin"

echo "🎤 Checking for Whisper model..."

mkdir -p "${WHISPER_MODEL_DIR}"
if [ ! -f "${WHISPER_MODEL_PATH}" ]; then
  echo "⬇️  Downloading Whisper model (ggml-base.en.bin)..."
  curl -L "${WHISPER_MODEL_URL}" -o "${WHISPER_MODEL_PATH}" || { echo "Download failed"; exit 1; }
else
  echo "Whisper model already present at ${WHISPER_MODEL_PATH}"
fi

# ---------------------------------------------------------

echo "Done."
