#!/usr/bin/env bash
set -euo pipefail

echo "📦 Installing test workload dependencies..."

export DEBIAN_FRONTEND=noninteractive
export APT_LISTCHANGES_FRONTEND=none

# --- Ensure apt can see everything we need (Ubuntu 'universe' often missing) ---
if ! apt-cache policy | grep -q universe; then
  echo "➕ Enabling 'universe' repo (Ubuntu)..."
  apt-get update -yq
  apt-get install -y --no-install-recommends software-properties-common || true
  add-apt-repository -y universe || true
fi

apt-get update -yq

# --- Common deps for your workloads/tests ---
apt-get install -y --no-install-recommends \
  git cmake build-essential \
  libopencv-dev \
  libsdl2-dev \
  libsdl2-ttf-dev \
  libsdl2-gfx-dev \
  libssl-dev \
  libcurl4-openssl-dev \
  python3-dev

# --- Try to install Paho via apt (preferred) ---
echo "🔎 Attempting apt install of Paho MQTT (C & C++)..."
if ! apt-get install -y --no-install-recommends libpaho-mqttpp3-dev libpaho-mqtt-dev libpaho-mqtt1.3; then
  echo "⚠️  Apt packages for Paho MQTT not found. Falling back to source build..."
fi

# --- If CMake config still not present, build Paho from source into ~/.local ---
PAHO_CONFIG_SYS="/usr/lib/cmake/PahoMqttCpp/PahoMqttCppConfig.cmake"
PAHO_PREFIX="${HOME}/.local"
PAHO_CONFIG_LOCAL="${PAHO_PREFIX}/lib/cmake/PahoMqttCpp/PahoMqttCppConfig.cmake"

if [ ! -f "$PAHO_CONFIG_SYS" ] && [ ! -f "$PAHO_CONFIG_LOCAL" ]; then
  echo "🔧 Building Paho MQTT (C & C++) from source into ${PAHO_PREFIX} ..."

  mkdir -p external

  # --- Paho C ---
  if [ ! -d external/paho.mqtt.c ]; then
    git clone --depth=1 https://github.com/eclipse/paho.mqtt.c.git external/paho.mqtt.c
  fi
  cmake -S external/paho.mqtt.c -B external/paho.mqtt.c/build \
    -DCMAKE_INSTALL_PREFIX="${PAHO_PREFIX}" \
    -DPAHO_ENABLE_TESTING=OFF \
    -DPAHO_WITH_SSL=OFF \
    -DPAHO_BUILD_STATIC=ON
  cmake --build external/paho.mqtt.c/build --target install -j"$(nproc)"

  # --- Paho C++ ---
  if [ ! -d external/paho.mqtt.cpp ]; then
    git clone --depth=1 https://github.com/eclipse/paho.mqtt.cpp.git external/paho.mqtt.cpp
  fi
  cmake -S external/paho.mqtt.cpp -B external/paho.mqtt.cpp/build \
    -DCMAKE_INSTALL_PREFIX="${PAHO_PREFIX}" \
    -DPAHO_MQTT_C_PATH="${PAHO_PREFIX}" \
    -DPAHO_BUILD_STATIC=ON
  cmake --build external/paho.mqtt.cpp/build --target install -j"$(nproc)"

  echo "✅ Paho MQTT installed to ${PAHO_PREFIX}"
fi

# --- Help CMake find Paho regardless of how it was installed ---
if [ -f "$PAHO_CONFIG_SYS" ]; then
  export CMAKE_PREFIX_PATH="/usr/lib/cmake/PahoMqttCpp:${CMAKE_PREFIX_PATH:-}"
  export PahoMqttCpp_DIR="/usr/lib/cmake/PahoMqttCpp"
elif [ -f "$PAHO_CONFIG_LOCAL" ]; then
  export CMAKE_PREFIX_PATH="${PAHO_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  export PahoMqttCpp_DIR="${PAHO_PREFIX}/lib/cmake/PahoMqttCpp"
fi

# --- Clean apt caches ---
apt-get clean
rm -rf /var/lib/apt/lists/*

echo "✅ Done installing test deps."
echo "ℹ️  Exports for this shell:"
echo "    PahoMqttCpp_DIR=${PahoMqttCpp_DIR:-<not set>}"
echo "    CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH:-<not set>}"
