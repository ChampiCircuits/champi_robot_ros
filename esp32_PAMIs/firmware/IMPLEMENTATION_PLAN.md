# Mini Robot Trajectory-Following Implementation Plan

## Goal
Implement firmware that drives a 2-wheel differential robot (2 stepper motors) along `EXPERIMENT_TRAJECTORY` from `include/generated_trajectory.h`, while keeping `src/main.cpp` lean and placing motion logic in `src/motion.cpp`.

Also integrate:
- HC-SR04 obstacle monitoring with pause/resume behavior.
- Team selection switch (blue/yellow) with trajectory mirroring for yellow.
- RGB LED status output for team and fault/waiting states.
- Tirette start switch: motion starts only after tirette event, then waits `DELAY_AFTER_PULL_CORD_S`.

## Scope and Constraints
- Board/framework: ESP32-C3 + Arduino (`platformio.ini`).
- Keep generated data as input-only (`include/generated_trajectory.h` should not be hand-edited in normal workflow).
- Preserve bring-up logs in `setup()` (`Serial.begin(115200)` and first boot print).
- Use non-blocking control in `loop()` (no long `delay()` once motion starts).
- Map size assumption: 3000 mm (X) x 2000 mm (Y).
- Mirror line for yellow team: vertical line `x = 1500 mm`.
- `DELAY_AFTER_PULL_CORD_S` is read from `include/generated_trajectory.h` and used as the pre-motion countdown after tirette start.

## Architecture (Code Organization)
- `src/main.cpp`
  - Owns lifecycle only: boot, init, periodic tick call.
  - Calls `motionInit()` in `setup()` and `motionTick(micros())` in `loop()`.
- `include/motion.h` (new)
  - Declares motion API and status structs/enums.
- `src/motion.cpp`
  - Owns trajectory follower, kinematics, pulse generation, HC-SR04 gating, team transform, and safety state machine.
- `include/generated_trajectory.h`
  - Provides `GLOBAL_SPEED_MM_S`, `DELAY_AFTER_PULL_CORD_S`, `Waypoint`, `EXPERIMENT_TRAJECTORY`, `TRAJECTORY_POINTS_COUNT`.

## Team Selection and Trajectory Transform
- Team input is read from a physical `0/1` switch (blue/yellow).
- Keep `EXPERIMENT_TRAJECTORY` immutable; build a runtime working trajectory buffer.
- Team can change while waiting for tirette; team is latched only when tirette start is triggered.
- If team is yellow, apply vertical symmetry around `x=1500 mm`:
  - `x_mirrored = 3000.0f - x_original`
  - `y_mirrored = y_original`
- Waypoint order remains unchanged; only coordinates are mirrored.
- Set LED to team color only after team is latched at tirette start:
  - Blue team -> blue LED
  - Yellow team -> yellow LED

## Data Flow and Control Loop
1. Init: configure GPIO (steppers, HC-SR04, team switch, tirette, RGB LED), reset follower state (`segment_index = 0`, `done = false`, `fault = none`), set state to wait for tirette.
2. Tick (fixed period, e.g. 5-10 ms):
   - While waiting for tirette, sample team switch continuously and update candidate team color on LED.
  - On tirette start edge: latch current team, build working trajectory (mirrored or original), and start a countdown timer from `DELAY_AFTER_PULL_CORD_S`.
   - During countdown, hold motors stopped.
   - After countdown expires, enter `RUNNING`.
   - Sample HC-SR04 distance and update obstacle state with hysteresis.
   - If obstacle is near, enter `PAUSED_OBSTACLE` and command zero wheel speed.
   - If obstacle clears, return to `RUNNING` and resume from current segment.
   - Read active segment: `P[i] -> P[i+1]`.
   - Compute segment direction and remaining distance.
   - Compute command `(v, omega)` using segment tracking (start with a simple P controller on heading/cross-track error).
   - Convert `(v, omega)` to wheel linear speeds:
     - `v_left = v - (omega * track_width / 2)`
     - `v_right = v + (omega * track_width / 2)`
   - Convert wheel mm/s -> steps/s with drivetrain constants.
   - Update stepper pulse scheduler (DIR + STEP toggles, no blocking).
3. Segment completion: if close enough to target waypoint, advance `segment_index`.
4. End condition: at last segment, stop both motors and report `completed`.

## State Machine and LED Policy
- `WAITING_TIRETTE`: robot not started yet; team can still change from switch; LED = orange.
- `START_DELAY`: tirette triggered, team latched, waiting `DELAY_AFTER_PULL_CORD_S`; LED = latched team color.
- `RUNNING`: follower active; LED = team color (blue or yellow).
- `PAUSED_OBSTACLE`: obstacle within stop threshold; LED stays team color.
- `FAULT`: invalid trajectory/sensor timeout/unexpected condition; LED = red and motors stopped.

## Implementation-Ready FSM (for `motion.cpp`)
Events:
- `EV_TICK(now_us)`: periodic control-loop update.
- `EV_TIRETTE_START_EDGE`: debounced tirette start edge.
- `EV_OBSTACLE_NEAR`: `distance_mm <= OBSTACLE_STOP_MM`.
- `EV_OBSTACLE_CLEAR`: `distance_mm >= OBSTACLE_RESUME_MM`.
- `EV_TRAJECTORY_DONE`: last waypoint reached.
- `EV_FAULT(code)`: unrecoverable condition.

Transitions:
| Current | Event | Guard | Next | Actions |
|---|---|---|---|---|
| `WAITING_TIRETTE` | `EV_TICK` | no tirette edge | `WAITING_TIRETTE` | sample team switch, update candidate team, LED orange, motors off |
| `WAITING_TIRETTE` | `EV_TIRETTE_START_EDGE` | team value valid | `START_DELAY` | latch team, build working trajectory (mirror if yellow), set `start_deadline_us = now + DELAY_AFTER_PULL_CORD_S`, LED team color, motors off |
| `START_DELAY` | `EV_TICK` | `now < start_deadline_us` | `START_DELAY` | keep motors off, LED team color |
| `START_DELAY` | `EV_TICK` | `now >= start_deadline_us` | `RUNNING` | enable follower execution from segment 0 |
| `RUNNING` | `EV_OBSTACLE_NEAR` | true | `PAUSED_OBSTACLE` | command zero wheel speed immediately |
| `RUNNING` | `EV_TRAJECTORY_DONE` | true | `COMPLETED` | stop motors, keep LED team color |
| `RUNNING` | `EV_FAULT(code)` | true | `FAULT` | stop motors, LED red, store fault code |
| `RUNNING` | `EV_TICK` | no obstacle/fault | `RUNNING` | execute tracking control and pulse generation |
| `PAUSED_OBSTACLE` | `EV_OBSTACLE_CLEAR` | true | `RUNNING` | resume from current segment |
| `PAUSED_OBSTACLE` | `EV_FAULT(code)` | true | `FAULT` | stop motors, LED red, store fault code |
| `PAUSED_OBSTACLE` | `EV_TICK` | obstacle still near | `PAUSED_OBSTACLE` | keep motors off |
| `COMPLETED` | `EV_TICK` | always | `COMPLETED` | keep motors off |
| `FAULT` | `EV_TICK` | always | `FAULT` | keep motors off, LED red |

Minimal runtime state fields:
- `MotionState state`
- `Team candidate_team` (updated while waiting)
- `Team latched_team` (frozen on tirette edge)
- `uint32_t start_deadline_us`
- `int segment_index`
- `bool obstacle_blocked`
- `float last_distance_mm`
- `FaultCode fault_code`

FSM invariants:
- Team switch is live only in `WAITING_TIRETTE`; after tirette edge, use `latched_team` only.
- No wheel motion command is allowed before `START_DELAY` expiry.
- All fault paths force motor stop and LED red.

## Required Drivetrain Parameters (Config)
Create a motion config block (compile-time constants) for:
- `WHEEL_DIAMETER_MM`
- `TRACK_WIDTH_MM`
- `MOTOR_STEPS_PER_REV`
- `MICROSTEPS`
- `GEAR_RATIO`
- `MAX_WHEEL_SPEED_MM_S`, `MAX_WHEEL_ACCEL_MM_S2`
- GPIO pins: left/right `STEP` and `DIR` (and optional `ENABLE`)

Add platform I/O and thresholds:
- HC-SR04 pins: `US_TRIG_PIN`, `US_ECHO_PIN`
- Obstacle thresholds: `OBSTACLE_STOP_MM`, `OBSTACLE_RESUME_MM` (hysteresis)
- Team switch pin: `TEAM_SWITCH_PIN`
- Tirette switch pin: `TIRETTE_PIN`
- RGB LED pins: `LED_R_PIN`, `LED_G_PIN`, `LED_B_PIN`
- Team/tirette debounce and edge timing: `TEAM_DEBOUNCE_MS`, `TIRETTE_DEBOUNCE_MS`

## Safety, Robustness, and Debug
- Guard invalid trajectory (`TRAJECTORY_POINTS_COUNT < 2`) -> fault + stop.
- Clamp speed and acceleration each tick.
- Watchdog-style stale loop check (if tick period is unexpectedly large, decelerate/stop).
- Emergency stop API (`motionEmergencyStop()`) that immediately disables pulse output.
- Validate ultrasonic reads (timeout/out-of-range) and choose safe fallback (pause or fault).
- Throttled serial telemetry (5-10 Hz): team, segment index, distance mm, motion state, wheel setpoints, fault code.

## Incremental Delivery Plan
1. **M1 - Motor Bring-up**
   - Implement low-level step pulse generation and direction control.
   - Validate fixed-speed forward/reverse on both wheels.
2. **M2 - Team Switch + LED Bring-up**
   - Read and debounce team switch.
   - Implement LED driver and verify pre-start color behavior.
3. **M3 - Tirette Start + Delay**
   - Detect tirette start edge.
  - Latch team at start and enforce `DELAY_AFTER_PULL_CORD_S` before motion.
4. **M4 - Straight Segment Execution**
   - Execute one segment `P0 -> P1` using open-loop speed and distance timing.
   - Confirm stop behavior and direction switching.
5. **M5 - Multi-Waypoint + Team Mirroring**
   - Iterate all trajectory points with segment transitions.
   - Apply yellow-team mirroring to runtime trajectory buffer.
6. **M6 - Obstacle Pause/Resume (HC-SR04)**
   - Add stop/resume gating with hysteresis and validate no rapid toggling.
7. **M7 - Closed-loop Path Tracking (odometry-based estimate)**
   - Add heading/cross-track correction (`v`, `omega`) and speed ramp limits.
8. **M8 - Fault Handling + Telemetry**
   - Finalize fault codes, emergency stop path, and debug outputs.

## Verification Workflow
- Build:
  - `pio run -d /home/etienne/champi_ws/src/champi_robot_ros/esp32_PAMIs/firmware`
- Flash:
  - `pio run -d /home/etienne/champi_ws/src/champi_robot_ros/esp32_PAMIs/firmware -t upload`
- Monitor:
  - `pio device monitor -d /home/etienne/champi_ws/src/champi_robot_ros/esp32_PAMIs/firmware -b 115200`

## Implementation Notes for Next Step
- Start with deterministic timing (`micros()` based scheduler) before adding sophisticated controller tuning.
- Keep `main.cpp` orchestration-only; all math/hardware logic belongs to `motion.cpp`.
- If `Waypoint` schema changes, update trajectory consumers and follower structs in the same commit.
- Keep team mirroring and obstacle policy in one state machine to avoid contradictory motor commands.
- Do not command any wheel motion before tirette start and `DELAY_AFTER_PULL_CORD_S` expiration.

