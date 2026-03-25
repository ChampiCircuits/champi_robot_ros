# AGENTS Guide

## Scope
- This repository is a single PlatformIO firmware project for ESP32-C3 (`esp32-c3-devkitc-02`) using Arduino.
- Core files are small; productivity depends on preserving the separation between generated trajectory data and runtime control code.

## Repository Map
- `platformio.ini`: one environment (`[env:esp32-c3-devkitc-02]`) with `platform = espressif32` and `framework = arduino`.
- `src/main.cpp`: boot/runtime entrypoints (`setup()`, `loop()`), currently initializes serial at 115200 and idles.
- `include/generated_trajectory.h`: auto-generated mission/config payload (`GLOBAL_SPEED_MM_S`, `Waypoint`, `EXPERIMENT_TRAJECTORY`).
- `src/motion.cpp`: currently empty, intended place for motion logic to keep `main.cpp` lean.

## Architecture and Data Flow
- Boot flow is Arduino-standard: `setup()` runs once, `loop()` repeats forever.
- Trajectory/config data is compile-time and header-defined, then included by runtime code (`#include "generated_trajectory.h"`).
- `EXPERIMENT_TRAJECTORY` is a fixed `const Waypoint[]`; point count is tracked separately by `TRAJECTORY_POINTS_COUNT`.
- Current behavior is a minimal firmware skeleton (`Serial.println("PAMI Booting...")`, then `delay(1000)`).

## Build and Device Workflow
- Build (verified in this repo):
  - `pio run -d /home/etienne/champi_ws/src/champi_robot_ros/esp32_PAMIs/firmware`
- Upload to board (same env):
  - `pio run -d /home/etienne/champi_ws/src/champi_robot_ros/esp32_PAMIs/firmware -t upload`
- Serial monitor at boot baud:
  - `pio device monitor -d /home/etienne/champi_ws/src/champi_robot_ros/esp32_PAMIs/firmware -b 115200`

## Project-Specific Conventions
- Treat `include/generated_trajectory.h` as generated input; prefer regenerating over manual edits when upstream tools exist.
- Keep hardware loop orchestration in `src/main.cpp`; move reusable motion/path logic into `src/motion.cpp`.
- Keep trajectory units consistent with existing names (`*_MM_S`, waypoint `x/y` in the same scale).
- When adding fields to `Waypoint` or trajectory constants, update both data definitions and runtime consumers in the same change.

## Practical Change Pattern
- Example feature path: implement waypoint iteration in `src/motion.cpp`, call it from `loop()` in `src/main.cpp`, and source points from `EXPERIMENT_TRAJECTORY`.
- Preserve early serial boot logs (`Serial.begin(115200)` + first print) because this is the primary bring-up/debug signal.
- Rebuild with `pio run` after each structural change; this catches signature/type drift quickly in this tiny codebase.

