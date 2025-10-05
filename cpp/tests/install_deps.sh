#!/usr/bin/env bash
set -euo pipefail

echo "📦 Installing test workload dependencies..."

export DEBIAN_FRONTEND=noninteractive
export APT_LISTCHANGES_FRONTEND=none

apt-get update -yq
apt-get install -y --no-install-recommends \
  libopencv-dev \
  libsdl2-dev \
  libsdl2-ttf-dev \
  libsdl2-gfx-dev \
  libssl-dev \
  libcurl4-openssl-dev \
  python3-dev

apt-get clean
rm -rf /var/lib/apt/lists/*

echo "✅ Done installing test deps."
