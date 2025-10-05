#!/usr/bin/env bash
set -euo pipefail

echo "📦 Installing test workload dependencies..."

export DEBIAN_FRONTEND=noninteractive
export APT_LISTCHANGES_FRONTEND=none

# -------------------------------------------------------------------
# 0) Optional pins for source builds (override via env if you like)
# -------------------------------------------------------------------
PAHO_C_TAG="${PAHO_C_TAG:-v1.3.13}"
PAHO_CPP_TAG="${PAHO_CPP_TAG:-v1.3.2}"
PAHO_PREFIX="${PAHO_PREFIX:-$HOME/.local}"

# -------------------------------------------------------------------
# 1) Ensure repos and common build deps
# -------------------------------------------------------------------
if command -v apt-get >/dev/null 2>&1; then
  # Enable 'universe' on Ubuntu minimal images
  if ! apt-cache policy | grep -q universe; then
    apt-get update -yq
    apt-get install -y --no-install-recommends software-properties-common || true
    add-apt-repository -y universe || true
  fi

  apt-get update -yq
  apt-get install -y --no-install-recommends \
    git cmake build-essential pkg-config \
    libopencv-dev \
    libsdl2-dev libsdl2-ttf-dev libsdl2-gfx-dev \
    libssl-dev \
    libcurl4-openssl-dev \
    python3-dev
else
  echo "❌ apt-get not found. This script assumes Debian/Ubuntu. Aborting."
  exit 1
fi

# -------------------------------------------------------------------
# 2) Try apt-first install for Paho C & C++
# -------------------------------------------------------------------
APT_PAHO_OK=true
echo "🔎 Attempting apt install of Paho MQTT (C & C++)..."
if ! apt-get install -y --no-install-recommends libpaho-mqtt-dev libpaho-mqttpp3-dev; then
  echo "⚠️  Paho apt packages not available."
  APT_PAHO_OK=false
fi

# Paths CMake uses when installed via apt
PAHO_CONFIG_SYS="/usr/lib/cmake/PahoMqttCpp/PahoMqttCppConfig.cmake"
ECLIPSE_PAHO_C_SYS="/usr/lib/cmake/eclipse-paho-mqtt-c/eclipse-paho-mqtt-c-config.cmake"

# -------------------------------------------------------------------
# 3) If apt didn't provide usable configs, build from source (SSL ON)
# -------------------------------------------------------------------
PAHO_CONFIG_LOCAL="${PAHO_PREFIX}/lib/cmake/PahoMqttCpp/PahoMqttCppConfig.cmake"
ECLIPSE_PAHO_C_LOCAL="${PAHO_PREFIX}/lib/cmake/eclipse-paho-mqtt-c/eclipse-paho-mqtt-c-config.cmake"

need_source_build=false
if [ "$APT_PAHO_OK" = "false" ]; then
  need_source_build=true
else
  # Even if apt install succeeded, ensure CMake configs are present
  if [ ! -f "$PAHO_CONFIG_SYS" ] || [ ! -f "$ECLIPSE_PAHO_C_SYS" ]; then
    need_source_build=true
  fi
fi

if [ "$need_source_build" = true ]; then
  echo "🔧 Building Paho MQTT from source into ${PAHO_PREFIX} …"
  mkdir -p external

  # --- Paho C (with SSL and both shared/static so *3as targets exist) ---
  if [ ! -d external/paho.mqtt.c ]; then
    git clone --depth=1 --branch "$PAHO_C_TAG" https://github.com/eclipse/paho.mqtt.c.git external/paho.mqtt.c
  fi
  cmake -S external/paho.mqtt.c -B external/paho.mqtt.c/build \
    -DCMAKE_INSTALL_PREFIX="${PAHO_PREFIX}" \
    -DPAHO_ENABLE_TESTING=OFF \
    -DPAHO_WITH_SSL=ON \
    -DPAHO_BUILD_SHARED=ON \
    -DPAHO_BUILD_STATIC=ON
  cmake --build external/paho.mqtt.c/build --target install -j"$(nproc)"

  # --- Paho C++ (finds eclipse-paho-mqtt-c via CMAKE_PREFIX_PATH) ---
  if [ ! -d external/paho.mqtt.cpp ]; then
    git clone --depth=1 --branch "$PAHO_CPP_TAG" https://github.com/eclipse/paho.mqtt.cpp.git external/paho.mqtt.cpp
  fi
  cmake -S external/paho.mqtt.cpp -B external/paho.mqtt.cpp/build \
    -DCMAKE_INSTALL_PREFIX="${PAHO_PREFIX}" \
    -DPAHO_BUILD_STATIC=ON
  cmake --build external/paho.mqtt.cpp/build --target install -j"$(nproc)"

  # After source install, prefer local configs
  export CMAKE_PREFIX_PATH="${PAHO_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  export PahoMqttCpp_DIR="${PAHO_PREFIX}/lib/cmake/PahoMqttCpp"
  export eclipse-paho-mqtt-c_DIR="${PAHO_PREFIX}/lib/cmake/eclipse-paho-mqtt-c"
else
  # Prefer system configs discovered via apt
  if [ -f "$PAHO_CONFIG_SYS" ]; then
    export CMAKE_PREFIX_PATH="/usr/lib/cmake/PahoMqttCpp:${CMAKE_PREFIX_PATH:-}"
    export PahoMqttCpp_DIR="/usr/lib/cmake/PahoMqttCpp"
  fi
  if [ -f "$ECLIPSE_PAHO_C_SYS" ]; then
    export CMAKE_PREFIX_PATH="/usr/lib/cmake/eclipse-paho-mqtt-c:${CMAKE_PREFIX_PATH:-}"
    export eclipse-paho-mqtt-c_DIR="/usr/lib/cmake/eclipse-paho-mqtt-c"
  fi
fi

# -------------------------------------------------------------------
# 4) Cleanup apt caches
# -------------------------------------------------------------------
apt-get clean
rm -rf /var/lib/apt/lists/*

echo "✅ Done installing test deps."
echo "ℹ️  Exports for this shell:"
echo "    CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH:-<not set>}"
echo "    PahoMqttCpp_DIR=${PahoMqttCpp_DIR:-<not set>}"
echo "    eclipse-paho-mqtt-c_DIR=${eclipse-paho-mqtt-c_DIR:-<not set>}"
