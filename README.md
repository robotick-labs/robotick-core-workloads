<img src="https://robotick.org/images/logo.png" style="display: block; width: 300px; border-radius: 12px;" />

<br/>

[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)

> **Work In Progress**
>
> This is a personal spare-time hobby codebase and not a production-ready project.
> The repo is public mainly for visibility, documentation, and easy sharing of experiments.
> Expect incomplete features, ongoing changes, and rough edges.

---

## 🤖 Overview

**Robotick** is a high-performance, modular C++ runtime for robotics and control systems - engineered for reliability, introspection, and composability across the full spectrum of platforms.

From bare-metal microcontrollers like the STM32 and ESP32 to Raspberry Pi, desktop systems, and edge-AI devices like NVIDIA Jetson (arm64), Robotick delivers real-time precision where it counts, and excellent performance everywhere else - without sacrificing ease of use or flexibility.

---

## 🚀 What is Robotick?

Robotick is a modern control engine designed to execute structured workloads with:

- Predictable, real-time performance
- Introspectable config, input, and output fields
- Zero-allocation memory buffer layout
- Modular composition and orchestration
- Full lifecycle management (load, start, tick, stop)
- Remote telemetry and future remote control

This is primarily a hobby codebase, but the runtime is being written clearly enough to be educational, inspectable, and reusable.

---

## ✨ Key Features

### 🧩 Modular Workloads

Each unit of logic is a workload - a small, testable module with clearly defined inputs, outputs, and config:

```cpp
struct HelloWorkload {
    HelloConfig config;
    HelloInputs inputs;
    HelloOutputs outputs;

    void tick(const TickInfo& tick_info);
};
```

Reflection macros make every field visible and usable for config, scripting, or telemetry.

### 🔁 Real-Time Engine

- Individual tick rates per workload
- Deterministic scheduling
- Ready for multithreaded execution
- Exceptionally consistent timing on MCUs
- Excellent latency characteristics on general-purpose platforms

### Composition System

Compose workloads into rich behaviours:

- `SequenceWorkload`: run workloads in order
- `SyncedPairWorkload`: parallel aligned ticking
- Future: event-driven, reactive, and conditional branching

### 🔬 Introspection & Reflection

Every field in config, inputs, and outputs is exposed at runtime, enabling:

- Remote configuration
- Schema-aware scripting
- Full field inspection for telemetry

No boilerplate. No fuss. Just structured access to everything that matters.

### 📡 Remote Telemetry & UI

Designed for live telemetry from the start:

- Structured data streaming
- Pluggable transport backends (UART, MQTT, etc.)
- UI- and scripting-ready introspection
- Foundation for remote tuning and remote control

### 🐍 Python Bindings

Use Python to:

- Orchestrate workload execution
- Inspect and modify live system state
- Visualise real-time data
- Integrate with scientific workflows

Zero-copy overlay support is in progress for high-efficiency interop.

#### Configuring the embedded Python runtime

`PythonWorkload` calls `ensure_python_runtime()` the first time it touches the interpreter. By default we import the standard `site` module and trust whatever user-level package directories exist on the host. If you need deterministic startup (skip `site`) or want to add workspace-specific module paths, configure the runtime up front:

```cpp
#include "robotick/systems/PythonRuntime.h"

using namespace robotick;

namespace
{
const char* const kPythonPaths[] = {"/home/robotick/workspaces/robotick-core-workloads/python"};
}

int main()
{
    PythonRuntimeConfig config;
    config.import_site = false;      // skip site packages for MCU determinism
    config.allow_user_site = false;  // ignore ~/.local/lib/python...
    config.extra_module_paths = kPythonPaths;
    config.extra_module_path_count = 1;
    set_python_runtime_config(config);

    // build/load Engine + workloads as usual
}
```

Pointers supplied via `extra_module_paths` must outlive the process (static arrays work well). You can also provide `post_init_hook` to run custom CPython setup while the GIL is held.

### 🧪 Simulation-First Testing

Built to be tested:

- Mockable physics interfaces
- Unit testable workloads
- Consistent simulated timing for verification
- Easy integration with CI

### ⚠️ Error Reporting + Metrics

Workloads should always pair log messages with a metric/output so operators (Hub/CLI) can see the same fault the log reports. The recommended pattern (also shown in `TemplateWorkload.cpp`) is:

```cpp
if (!sensor_ok)
{
    ROBOTICK_WARNING("sensor '%s' below threshold (%d < %d)", inputs.sensor_label.c_str(), inputs.sensor_value, config.threshold);
    outputs.status = "sensor below threshold";
    outputs.warning_count++; // telemetry counter surfaced to Hub
}
```

Use `ROBOTICK_INFO/WARNING/ERROR` depending on severity, and update an output/telemetry counter each time to keep systems observable.

## Coordinate Convention

Robotick control and orientation signals follow ROS REP-103:

- Right-handed orientation convention.
- `+X` is forward.
- `+Y` is left.
- `+Z` is up.
- Positive yaw / `angular_speed` is counter-clockwise when viewed from above (rotation about `+Z`).
- Roll/pitch/yaw are rotations about `+X`, `+Y`, and `+Z` respectively.

---

## 📁 Project Structure

```bash
robotick/
├── cpp/                    # Core engine and built-in workloads
├── python/                 # Python bindings (PyBind11)
├── tests/                  # Unit test suite (Catch2)
└── tools/                  # Dev tools and profilers
```

---

## 🛠️ Platform Support

- STM32 (e.g. B-G431B-ESC1)
- Raspberry Pi (Pi 2 → Pi 5)
- Desktop (Windows / Linux)
- Jetson (Nano, Orin) via arm64

Compiled and deployed executables are tested across an expanding range of targets.

---

## 🎯 Design Principles

- **Structured**: Everything is inspectable, composable, and testable
- **Reliable**: High-precision real-time behaviour on real hardware
- **Performant**: Zero-copy, buffer-based layout for speed and clarity
- **Accessible**: Easy for learners, powerful for researchers
- **Flexible**: Modular composition and optional Python integration
- **Embeddable**: Works standalone or embedded into larger stacks

---

## 🗺️ Things I'm Experimenting With

- [ ] Python zero-copy memory overlay
- [ ] Live telemetry viewer (web + Python)
- [ ] Graphical workload visualiser
- [ ] Composition scripting and editor
- [ ] ROS2 integration bridge
- [ ] Official templates for STM32, Pi, Jetson

---

## 📄 License

Licensed under the **Apache 2.0 License** – free to use, adapt, and build upon.

---

## 💬 Feedback

This repository is public so people can follow along, but I’m keeping it as a simple one-person project for now and am not currently taking on outside bug reports or contribution flow.

Visit [https://robotick.org](https://robotick.org) for broader project context.
