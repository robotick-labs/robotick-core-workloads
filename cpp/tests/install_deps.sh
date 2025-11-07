#!/usr/bin/env bash
set -euo pipefail

echo "📦 Installing Robotick test workload dependencies..."

export DEBIAN_FRONTEND=noninteractive
export APT_LISTCHANGES_FRONTEND=none

apt-get update -yq
apt-get install -y --no-install-recommends \
  git cmake build-essential pkg-config \
  libopencv-dev \
  libsdl2-dev libsdl2-ttf-dev libsdl2-gfx-dev \
  libssl-dev libcurl4-openssl-dev \
  python3-dev \
  libyaml-cpp-dev \
  libkissfft-dev

apt-get clean
rm -rf /var/lib/apt/lists/*

echo "✅ Done."
