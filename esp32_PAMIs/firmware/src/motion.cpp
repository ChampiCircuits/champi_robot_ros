#include <Arduino.h>
#include <math.h>

#include "generated_trajectory.h"
#include "led_status.h"
#include "config.h"
#include "motion.h"
#include "stepper_control.h"
#include "logging.h"

namespace {

using namespace Config;

// Simple bench test mode: set to true to spin both motors forward at fixed speed.
bool g_forward_test_mode_enabled = false;
float g_forward_test_speed_mm_s = -1000.0f;

enum class SegmentPhase : uint8_t {
    IDLE = 0,
    TURNING,
    DRIVING,
    WAITING_POINT,
};

struct DebouncedInput {
    bool stable_value;
    bool raw_last;
    uint32_t last_change_ms;
};

MotionState g_state = MotionState::WAITING_TIRETTE;
Team g_candidate_team = Team::BLUE;
bool g_blocked_by_obstacle = false;
bool g_forward_test_mode_was_active = false;
int g_segment_index = 0;
uint32_t g_start_deadline_us = 0;
uint32_t g_last_telemetry_ms = 0;
uint32_t g_phase_deadline_us = 0;
uint32_t g_pending_drive_duration_us = 0;
uint32_t g_pause_started_us = 0;
float g_estimated_heading_rad = 0.0f;
float g_target_heading_rad = 0.0f;
float g_drive_speed_mm_s = GLOBAL_SPEED_MM_S;
float g_cmd_left_mm_s = 0.0f;
float g_cmd_right_mm_s = 0.0f;
SegmentPhase g_segment_phase = SegmentPhase::IDLE;

// Accel/decel ramp state (DRIVING phase only).
enum class DriveRampPhase : uint8_t { ACCEL, CRUISE, DECEL };
float g_current_drive_speed_mm_s = 0.0f;   // actual commanded speed (ramp output)
float g_drive_distance_traveled_mm = 0.0f; // integrated distance within current segment
float g_drive_segment_len_mm = 0.0f;       // length of the current drive segment
uint32_t g_last_drive_tick_us = 0;         // last tick timestamp for dt computation
DriveRampPhase g_drive_ramp_phase = DriveRampPhase::ACCEL;

Waypoint g_working_trajectory[TRAJECTORY_POINTS_COUNT];
bool g_match_timeout_armed = false;
bool g_match_timeout_triggered = false;
uint32_t g_match_timeout_deadline_us = 0;

constexpr uint32_t kMatchHardStopUs = 100000000UL; // 100 s after pull-cord start.

constexpr int kGeneratedTrajectoryPoints = static_cast<int>(sizeof(EXPERIMENT_TRAJECTORY) / sizeof(EXPERIMENT_TRAJECTORY[0]));

int activeTrajectoryPoints() {
    return (kGeneratedTrajectoryPoints < TRAJECTORY_POINTS_COUNT) ? kGeneratedTrajectoryPoints : TRAJECTORY_POINTS_COUNT;
}

bool g_tirette_input = false;

bool timeReachedUs(uint32_t now_us, uint32_t deadline_us) {
    return static_cast<int32_t>(now_us - deadline_us) >= 0;
}

void applyLedPolicy() {
    ledStatusApply(g_state, g_candidate_team);
}

bool readDigitalActive(int pin, bool pullup_enabled) {
    const int level = digitalRead(pin);
    if (pullup_enabled) {
        return level == LOW;
    }
    return level == HIGH;
}

bool updateDebounced(DebouncedInput &input, bool raw_value, uint32_t now_ms, uint32_t debounce_ms) {
    if (raw_value != input.raw_last) {
        input.raw_last = raw_value;
        input.last_change_ms = now_ms;
    }

    const bool expired = static_cast<uint32_t>(now_ms - input.last_change_ms) >= debounce_ms;
    if (expired && input.stable_value != input.raw_last) {
        input.stable_value = input.raw_last;
        return true;
    }
    return false;
}

void buildWorkingTrajectory(Team team) {
    const int points = activeTrajectoryPoints();
    for (int i = 0; i < points; ++i) {
        Waypoint wp = EXPERIMENT_TRAJECTORY[i];
        if (team == Team::YELLOW) {
            wp.x = MAP_WIDTH_MM - wp.x;
            float mirrored_heading = 180.0f - wp.headingDeg;
            while (mirrored_heading < 0.0f) {
                mirrored_heading += 360.0f;
            }
            while (mirrored_heading >= 360.0f) {
                mirrored_heading -= 360.0f;
            }
            wp.headingDeg = mirrored_heading;
        }
        g_working_trajectory[i] = wp;
    }
}

float distanceBetween(const Waypoint &a, const Waypoint &b) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    return sqrtf((dx * dx) + (dy * dy));
}

float normalizeAngle(float angle_rad) {
    while (angle_rad > static_cast<float>(PI)) {
        angle_rad -= static_cast<float>(2.0 * PI);
    }
    while (angle_rad < static_cast<float>(-PI)) {
        angle_rad += static_cast<float>(2.0 * PI);
    }
    return angle_rad;
}

float angleBetween(const Waypoint &from, const Waypoint &to) {
    return atan2f(to.y - from.y, to.x - from.x);
}

bool startNextSegment(const uint32_t now_us) {
    const int points = activeTrajectoryPoints();
    while (g_segment_index < (points - 1)) {
        const Waypoint &from = g_working_trajectory[g_segment_index];
        const Waypoint &to = g_working_trajectory[g_segment_index + 1];
        const float segment_len_mm = distanceBetween(from, to);
        if (segment_len_mm < 1.0f) {
            ++g_segment_index;
            continue;
        }

        g_target_heading_rad = angleBetween(from, to);
        const float delta_heading = normalizeAngle(g_target_heading_rad - g_estimated_heading_rad);
        const float abs_delta = fabsf(delta_heading);

        const float drive_duration_s = segment_len_mm / g_drive_speed_mm_s;
        g_pending_drive_duration_us = static_cast<uint32_t>(drive_duration_s * 1000000.0f);

        if (abs_delta >= MIN_TURN_RAD) {
            const float turn_omega_rad_s = (ANGULAR_SPEED_RAD_S > 0.0f)
                                           ? ANGULAR_SPEED_RAD_S
                                           : (2.0f * TURN_WHEEL_SPEED_MM_S) / ENTRAXE_MM;
            const float turn_duration_s = abs_delta / turn_omega_rad_s;
            g_phase_deadline_us = now_us + static_cast<uint32_t>(turn_duration_s * 1000000.0f);

            const float turn_wheel_speed_mm_s = turn_omega_rad_s * ENTRAXE_MM * 0.5f;

            if (delta_heading > 0.0f) {
                g_cmd_left_mm_s = -turn_wheel_speed_mm_s;
                g_cmd_right_mm_s = turn_wheel_speed_mm_s;
            } else {
                g_cmd_left_mm_s = turn_wheel_speed_mm_s;
                g_cmd_right_mm_s = -turn_wheel_speed_mm_s;
            }
            g_drive_segment_len_mm = segment_len_mm;  // stored for DRIVE init after turn
            g_segment_phase = SegmentPhase::TURNING;
        } else {
            // No turn needed — start DRIVING directly with ramp from zero.
            g_estimated_heading_rad = g_target_heading_rad;
            g_drive_segment_len_mm = segment_len_mm;
            g_drive_distance_traveled_mm = 0.0f;
            g_current_drive_speed_mm_s = 0.0f;
            g_drive_ramp_phase = DriveRampPhase::ACCEL;
            g_last_drive_tick_us = now_us;
            g_phase_deadline_us = 0;
            g_cmd_left_mm_s = 0.0f;
            g_cmd_right_mm_s = 0.0f;
            g_segment_phase = SegmentPhase::DRIVING;
        }
        {
            const float d_accel = (g_drive_speed_mm_s * g_drive_speed_mm_s) / (2.0f * ACCEL_MM_S2);
            const float d_decel = (g_drive_speed_mm_s * g_drive_speed_mm_s) / (2.0f * DECEL_MM_S2);
            LOG_WARN("Motion", "[seg] %d->%d len=%.1fmm turn=%.1fdeg drive=%.1fs target=%.1fmm/s d_accel=%.1fmm d_decel=%.1fmm",
                     g_segment_index, g_segment_index + 1, segment_len_mm,
                     delta_heading * 180.0 / M_PI, drive_duration_s,
                     g_drive_speed_mm_s, d_accel, d_decel);
        }
        return true;
    }
    return false;
}

bool startWaypointWaitIfNeeded(const uint32_t now_us) {
    const int points = activeTrajectoryPoints();
    if (g_segment_index < 0 || g_segment_index >= points) {
        return false;
    }

    const float wait_s = g_working_trajectory[g_segment_index].waitS;
    if (wait_s <= 0.0f) {
        return false;
    }

    g_phase_deadline_us = now_us + static_cast<uint32_t>(wait_s * 1000000.0f);
    g_cmd_left_mm_s = 0.0f;
    g_cmd_right_mm_s = 0.0f;
    g_segment_phase = SegmentPhase::WAITING_POINT;
    return true;
}

void resetRunProgress() {
    g_segment_index = 0;
    g_phase_deadline_us = 0;
    g_pending_drive_duration_us = 0;
    g_pause_started_us = 0;
    g_target_heading_rad = 0.0f;
    g_cmd_left_mm_s = 0.0f;
    g_cmd_right_mm_s = 0.0f;
    g_segment_phase = SegmentPhase::IDLE;
    g_current_drive_speed_mm_s = 0.0f;
    g_drive_distance_traveled_mm = 0.0f;
    g_drive_segment_len_mm = 0.0f;
    g_last_drive_tick_us = 0;
    g_drive_ramp_phase = DriveRampPhase::ACCEL;

    // Set initial heading from the first waypoint orientation.
    const int points = activeTrajectoryPoints();
    if (points >= 1) {
        g_estimated_heading_rad = g_working_trajectory[0].headingDeg * static_cast<float>(M_PI) / 180.0f;
    } else {
        g_estimated_heading_rad = 0.0f;
    }
}

void commandWheelSpeeds(float left_mm_s, float right_mm_s, uint32_t now_us) {
    stepperControlCommandMmS(left_mm_s, right_mm_s, now_us);
}

float readUltrasonicDistanceMm() {
    digitalWrite(US_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG_PIN, LOW);

    const unsigned long pulse_us = pulseIn(US_ECHO_PIN, HIGH, US_TIMEOUT_US);
    if (pulse_us == 0) {
        return -1.0f;
    }

    // HC-SR04: distance_cm = pulse_us / 58.0
    return (static_cast<float>(pulse_us) / 58.3f) * 10.0f;
}

struct RemainingEstimate {
    float distance_mm;
    float angle_rad;
};

float segmentLengthMm(int segment_index) {
    const int points = activeTrajectoryPoints();
    if (segment_index < 0 || segment_index >= (points - 1)) {
        return 0.0f;
    }
    return distanceBetween(g_working_trajectory[segment_index], g_working_trajectory[segment_index + 1]);
}

RemainingEstimate estimateCurrentSegmentRemaining(uint32_t now_us) {
    RemainingEstimate estimate{0.0f, 0.0f};
    const int points = activeTrajectoryPoints();

    if (points < 2 || g_segment_index >= (points - 1)) {
        return estimate;
    }

    const float current_seg_len_mm = segmentLengthMm(g_segment_index);
    if (current_seg_len_mm < 1.0f) {
        return estimate;
    }

    const Waypoint &from = g_working_trajectory[g_segment_index];
    const Waypoint &to = g_working_trajectory[g_segment_index + 1];
    const float segment_target_heading = angleBetween(from, to);
    const float total_turn_rad = fabsf(normalizeAngle(segment_target_heading - g_estimated_heading_rad));

    // When paused, pretend time stopped at pause start
    const uint32_t effective_now_us = (g_state == MotionState::PAUSED_OBSTACLE && g_pause_started_us != 0)
                                      ? g_pause_started_us : now_us;

    if (g_segment_phase == SegmentPhase::TURNING) {
        estimate.distance_mm = current_seg_len_mm;

        float remaining_turn_rad = total_turn_rad;
        if (g_phase_deadline_us != 0 && !timeReachedUs(effective_now_us, g_phase_deadline_us)) {
            const float remaining_turn_s = static_cast<float>(g_phase_deadline_us - effective_now_us) / 1000000.0f;
            const float turn_omega_rad_s = (ANGULAR_SPEED_RAD_S > 0.0f)
                                           ? ANGULAR_SPEED_RAD_S
                                           : (2.0f * TURN_WHEEL_SPEED_MM_S) / ENTRAXE_MM;
            remaining_turn_rad = remaining_turn_s * turn_omega_rad_s;
            if (remaining_turn_rad > total_turn_rad) {
                remaining_turn_rad = total_turn_rad;
            }
        } else {
            remaining_turn_rad = 0.0f;
        }

        estimate.angle_rad = remaining_turn_rad;
        return estimate;
    }

    if (g_segment_phase == SegmentPhase::DRIVING) {
        // Distance-based estimate: directly uses integrated travel distance.
        float remaining = g_drive_segment_len_mm - g_drive_distance_traveled_mm;
        if (remaining < 0.0f) { remaining = 0.0f; }
        estimate.distance_mm = remaining;
        estimate.angle_rad = 0.0f;
        return estimate;
    }

    if (g_segment_phase == SegmentPhase::WAITING_POINT) {
        estimate.distance_mm = 0.0f;
        estimate.angle_rad = 0.0f;
        return estimate;
    }

    estimate.distance_mm = current_seg_len_mm;
    if (total_turn_rad >= MIN_TURN_RAD) {
        estimate.angle_rad = total_turn_rad;
    }
    return estimate;
}

void publishTelemetry(uint32_t now_ms, uint32_t now_us, float distance_mm) {
    if (static_cast<uint32_t>(now_ms - g_last_telemetry_ms) < TELEMETRY_PERIOD_MS) {
        return;
    }
    g_last_telemetry_ms = now_ms;

    const char *state = "UNKNOWN";
    switch (g_state) {
        case MotionState::WAITING_TIRETTE: state = "WAITING_TIRETTE"; break;
        case MotionState::START_DELAY: state = "START_DELAY"; break;
        case MotionState::RUNNING: state = "RUNNING"; break;
        case MotionState::PAUSED_OBSTACLE: state = "PAUSED_OBSTACLE"; break;
        case MotionState::COMPLETED: state = "COMPLETED"; break;
        case MotionState::FAULT: state = "FAULT"; break;
    }

    const char *phase = "IDLE";
    switch (g_segment_phase) {
        case SegmentPhase::IDLE: phase = "IDLE"; break;
        case SegmentPhase::TURNING: phase = "TURN"; break;
        case SegmentPhase::DRIVING: phase = "DRIVE"; break;
        case SegmentPhase::WAITING_POINT: phase = "WAIT"; break;
    }

    const RemainingEstimate remaining = estimateCurrentSegmentRemaining(now_us);
    const int points = activeTrajectoryPoints();
    const int max_segment_index = (points > 0) ? (points - 1) : 0;
    const int segment_from = (g_segment_index < max_segment_index) ? g_segment_index : max_segment_index;
    const int segment_to = (segment_from < max_segment_index) ? (segment_from + 1) : segment_from;
    const float avg_speed_mm_s = (fabsf(g_cmd_left_mm_s) + fabsf(g_cmd_right_mm_s)) / 2.0f;

    // Use pause-frozen time for turn/wait deadline display (DRIVING no longer uses deadline).
    const uint32_t effective_now_us = (g_state == MotionState::PAUSED_OBSTACLE && g_pause_started_us != 0)
                                      ? g_pause_started_us : now_us;
    unsigned long remaining_phase_ms = 0UL;
    if (g_phase_deadline_us != 0 && !timeReachedUs(effective_now_us, g_phase_deadline_us)) {
        remaining_phase_ms = static_cast<unsigned long>((g_phase_deadline_us - effective_now_us) / 1000);
    }

    const char* ramp_phase_name = "N/A";
    if (g_segment_phase == SegmentPhase::DRIVING) {
        switch (g_drive_ramp_phase) {
            case DriveRampPhase::ACCEL:  ramp_phase_name = "ACCEL";  break;
            case DriveRampPhase::CRUISE: ramp_phase_name = "CRUISE"; break;
            case DriveRampPhase::DECEL:  ramp_phase_name = "DECEL";  break;
        }
    }

    Serial.printf("[motion] state=%s phase=%s ramp=%s team=%s seg=%d->%d spd_cmd=%.1f ramp_spd=%.1f traveled_mm=%.1f seg_rem_mm=%.1f seg_rem_deg=%.1f obstacle=%d t_phase_ms=%lu us_mm=%.1f\n",
                  state,
                  phase,
                  ramp_phase_name,
                  g_candidate_team == Team::YELLOW ? "YELLOW" : "BLUE",
                  segment_from,
                  segment_to,
                  avg_speed_mm_s,
                  g_current_drive_speed_mm_s,
                  g_drive_distance_traveled_mm,
                  remaining.distance_mm,
                  remaining.angle_rad * 180.0f / static_cast<float>(PI),
                  g_blocked_by_obstacle ? 1 : 0,
                  remaining_phase_ms,
                  distance_mm);
}

void stopMotors() {
    stepperControlStop();
}

void enforceHardMotorStop() {
    stopMotors();
    digitalWrite(ENABLE_MOTORS, HIGH); // Disable drivers to avoid heating and any further motion.
}

void printStepperDiagnostics() {
    const float wheel_circumference_mm = static_cast<float>(PI) * WHEEL_DIAMETER_MM;
    const float steps_per_wheel_turn = MOTOR_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO;
    const float steps_per_mm = steps_per_wheel_turn / wheel_circumference_mm;

    LOG_WARN("Stepper", "=== Diagnostics ===");
    LOG_WARN("Stepper", "WHEEL_DIAMETER_MM=%.1f", WHEEL_DIAMETER_MM);
    LOG_WARN("Stepper", "MOTOR_STEPS_PER_REV=%.0f", MOTOR_STEPS_PER_REV);
    LOG_WARN("Stepper", "MICROSTEPS=%.0f", MICROSTEPS);
    LOG_WARN("Stepper", "GEAR_RATIO=%.1f", GEAR_RATIO);
    LOG_WARN("Stepper", "wheel_circumference_mm=%.2f", wheel_circumference_mm);
    LOG_WARN("Stepper", "steps_per_wheel_turn=%.0f", steps_per_wheel_turn);
    LOG_WARN("Stepper", "steps_per_mm=%.2f", steps_per_mm);
    LOG_WARN("Stepper", "For 80 mm/s: %.0f steps/s, period=%.0f µs", 80.0f * steps_per_mm, 1000000.0f / (80.0f * steps_per_mm * 2.0f));
    LOG_WARN("Stepper", "For 200 mm/s: %.0f steps/s, period=%.0f µs", 200.0f * steps_per_mm, 1000000.0f / (200.0f * steps_per_mm * 2.0f));
    LOG_WARN("Stepper", "=== Ramp profile ===");
    LOG_WARN("Stepper", "ACCEL=%.0fmm/s²  DECEL=%.0fmm/s²  target=%.0fmm/s", ACCEL_MM_S2, DECEL_MM_S2, GLOBAL_SPEED_MM_S);
    const float d_accel_full = (GLOBAL_SPEED_MM_S * GLOBAL_SPEED_MM_S) / (2.0f * ACCEL_MM_S2);
    const float d_decel_full = (GLOBAL_SPEED_MM_S * GLOBAL_SPEED_MM_S) / (2.0f * DECEL_MM_S2);
    LOG_WARN("Stepper", "Full ramp: accel_dist=%.1fmm  decel_dist=%.1fmm  min_seg_for_cruise=%.1fmm",
             d_accel_full, d_decel_full, d_accel_full + d_decel_full);
}

} // namespace

void motionInit() {
    LOG_INFO("Motion", "Initializing system...");
    pinMode(TEAM_SWITCH_PIN, TEAM_SWITCH_PULLUP ? INPUT_PULLUP : INPUT);
    // pinMode(TIRETTE_PIN, ANALOG);

    // pinMode(33, OUTPUT);
    // digitalWrite(33, HIGH); // Pull-up for TIRETTE 

    pinMode(US_TRIG_PIN, OUTPUT);
    pinMode(US_ECHO_PIN, INPUT);
    LOG_INFO("Motion", "Initialized pins...");
    ledStatusInit();
    LOG_INFO("Motion", "Initialized status LED...");
    stepperControlInit();
    pinMode(ACTUATOR_PIN, ANALOG);
    analogWrite(ACTUATOR_PIN, 0);

    stopMotors();
    LOG_INFO("Motion", "Initialized stepperControl...");

    {
        // // TEST ACTUATOR
        // for (int v = 0; v <= 255; v += 5) {
        //     analogWrite(ACTUATOR_PIN, v);
        //     delay(100);
        //     Serial.printf("Actuator test: value=%d\n", v);
        // }
        // analogWrite(ACTUATOR_PIN, 0);
        // while (1) {}
    }

    {
        // TEST US SENSOR
        // while (true)
        // {
        //     const auto distance_mm = readUltrasonicDistanceMm();
        //     if (distance_mm > 0.0f) {
        //         if (distance_mm <= OBSTACLE_STOP_MM) {
        //             g_blocked_by_obstacle = true;
        //         } else if (distance_mm >= OBSTACLE_RESUME_MM) {
        //             g_blocked_by_obstacle = false;
        //         }
        //     }
        //     LOG_INFO("Motion", "distance_mm=%.1f, blocked? %d", distance_mm, g_blocked_by_obstacle);
        // }
    }

    // TEST CYCLE THROUGH COLORS
    {
        // while (true)
        // {
        //     setLedRgb(255, 30, 0);
        //     sleep(2);
        //     setLedRgb(255, 120, 0);
        //     sleep(2);
        // }
    }

    // TEST TIRETTE
    {
        // while (true)
        // {
        //     bool team = digitalRead(TEAM_SWITCH_PIN);
        //     int tirette = analogRead(TIRETTE_PIN);
        //     // LOG_INFO("tirette", "tirette %d team %d", tirette, team);
        //     Serial.println("=== TEST TIRETTE ===");
        //     Serial.printf("TTIRETTE_PIN=%d (raw=%d)\n", TIRETTE_PIN, tirette);
        //     Serial.println("===================");
        //     delay(500);
        // }
    }

    const bool team_raw = readDigitalActive(TEAM_SWITCH_PIN, TEAM_SWITCH_PULLUP);
    int tirette_raw = analogRead(TIRETTE_PIN);
    // g_tirette_input = DebouncedInput{tirette_raw, tirette_raw, millis()};
    g_tirette_input = (tirette_raw > 512); 
    Serial.printf("tirette_raw=%d", tirette_raw);
    Serial.println();

    g_candidate_team = team_raw ? Team::YELLOW : Team::BLUE;
    g_state = MotionState::WAITING_TIRETTE;
    g_blocked_by_obstacle = false;
    g_match_timeout_armed = false;
    g_match_timeout_triggered = false;
    g_match_timeout_deadline_us = 0;
    resetRunProgress();
    g_start_deadline_us = 0;
    g_last_telemetry_ms = 0;

    applyLedPolicy();

    if (kGeneratedTrajectoryPoints != TRAJECTORY_POINTS_COUNT) {
        LOG_WARN("Motion", "Trajectory count mismatch: declared=%d generated=%d (using %d)",
                 TRAJECTORY_POINTS_COUNT, kGeneratedTrajectoryPoints, activeTrajectoryPoints());
    }

    if (g_forward_test_mode_enabled) {
        LOG_WARN("Motion", "Forward TEST mode enabled: left=right=%.1f mm/s", g_forward_test_speed_mm_s);
    }

    printStepperDiagnostics();
}

void motionTick(uint32_t now_us) {
    const uint32_t now_ms = millis();

    if (g_match_timeout_armed && !g_match_timeout_triggered && timeReachedUs(now_us, g_match_timeout_deadline_us)) {
        g_match_timeout_triggered = true;
        g_state = MotionState::COMPLETED;
        LOG_WARN("Motion", "Hard stop timeout reached (100s after pull-cord). Motors disabled permanently.");
    }

    if (g_match_timeout_triggered) {
        enforceHardMotorStop();
        applyLedPolicy();
        analogWrite(ACTUATOR_PIN, 100);
        return;
    }

    if (g_forward_test_mode_enabled) {
        g_forward_test_mode_was_active = true;
        commandWheelSpeeds(g_forward_test_speed_mm_s, g_forward_test_speed_mm_s, now_us);
        stepperControlTick(now_us);
        // publishTelemetry(now_ms, now_us, -1.0f);
        return;
    }

    if (g_forward_test_mode_was_active) {
        stopMotors();
        g_forward_test_mode_was_active = false;
    }

    // const bool tirette_raw = readDigitalActive(TIRETTE_PIN, TIRETTE_PULLUP);
    // const bool tirette_changed = updateDebounced(g_tirette_input, tirette_raw, now_ms, TIRETTE_DEBOUNCE_MS);
    // const bool tirette_start_edge = tirette_changed && g_tirette_input.stable_value;
    // const bool tirette_start_edge = false; // TODO
    int tirette_raw = analogRead(TIRETTE_PIN);
    const bool tirette_start_edge = (tirette_raw > 512);
    // Serial.printf("tirette_raw=%d", tirette_raw);
    // Serial.println();

    float distance_mm = -1.0f;
    // FIXME: pulseIn blocks up to 25ms and kills step generation at high speed.
    distance_mm = readUltrasonicDistanceMm();
    if (distance_mm > 0.0f) {
        if (distance_mm <= OBSTACLE_STOP_MM) {
            g_blocked_by_obstacle = true;
        } else if (distance_mm >= OBSTACLE_RESUME_MM) {
            g_blocked_by_obstacle = false;
        }
    }

    switch (g_state) {
        case MotionState::WAITING_TIRETTE:
            {
                const bool team_raw = readDigitalActive(TEAM_SWITCH_PIN, TEAM_SWITCH_PULLUP);
                g_candidate_team = team_raw ? Team::YELLOW : Team::BLUE;
                enforceHardMotorStop();
                if (tirette_start_edge) {
                    buildWorkingTrajectory(g_candidate_team);
                    resetRunProgress();
                    g_start_deadline_us = now_us + static_cast<uint32_t>(DELAY_AFTER_PULL_CORD_S * 1000000.0f);
                    g_match_timeout_armed = true;
                    g_match_timeout_deadline_us = now_us + kMatchHardStopUs;
                    g_state = MotionState::START_DELAY;
                }
                break;
            }
        case MotionState::START_DELAY:
            {
                enforceHardMotorStop();
                if (timeReachedUs(now_us, g_start_deadline_us)) {
                    if (!startWaypointWaitIfNeeded(now_us) && !startNextSegment(now_us)) {
                        g_state = MotionState::COMPLETED;
                        break;
                    }
                    g_state = MotionState::RUNNING;
                }
                break;
            }
        case MotionState::RUNNING:
            {
                if (g_blocked_by_obstacle && g_segment_phase == SegmentPhase::DRIVING) {
                    stopMotors();
                    g_pause_started_us = now_us;
                    LOG_WARN("Motion", "[obstacle] Pausing mid-segment: traveled=%.1fmm rem=%.1fmm spd=%.1fmm/s",
                             g_drive_distance_traveled_mm,
                             g_drive_segment_len_mm - g_drive_distance_traveled_mm,
                             g_current_drive_speed_mm_s);
                    g_state = MotionState::PAUSED_OBSTACLE;
                } else {
                    // --- Accel/decel ramp for DRIVING phase ---
                    if (g_segment_phase == SegmentPhase::DRIVING) {
                        float dt_s = static_cast<float>(now_us - g_last_drive_tick_us) / 1000000.0f;
                        if (dt_s > 0.05f) { dt_s = 0.05f; }  // cap dt to avoid jump on first tick

                        const float remaining_mm = g_drive_segment_len_mm - g_drive_distance_traveled_mm;
                        const float d_stop = (g_current_drive_speed_mm_s * g_current_drive_speed_mm_s)
                                             / (2.0f * DECEL_MM_S2);

                        DriveRampPhase new_ramp_phase;
                        if (remaining_mm <= d_stop + 1.0f) {
                            new_ramp_phase = DriveRampPhase::DECEL;
                        } else if (g_current_drive_speed_mm_s < g_drive_speed_mm_s) {
                            new_ramp_phase = DriveRampPhase::ACCEL;
                        } else {
                            new_ramp_phase = DriveRampPhase::CRUISE;
                        }

                        if (new_ramp_phase != g_drive_ramp_phase) {
                            const char* phase_names[] = {"ACCEL", "CRUISE", "DECEL"};
                            LOG_WARN("Motion", "[ramp] %s -> %s  spd=%.1fmm/s  rem=%.1fmm  d_stop=%.1fmm  traveled=%.1fmm",
                                     phase_names[static_cast<int>(g_drive_ramp_phase)],
                                     phase_names[static_cast<int>(new_ramp_phase)],
                                     g_current_drive_speed_mm_s, remaining_mm, d_stop,
                                     g_drive_distance_traveled_mm);
                            g_drive_ramp_phase = new_ramp_phase;
                        }

                        if (new_ramp_phase == DriveRampPhase::DECEL) {
                            g_current_drive_speed_mm_s -= DECEL_MM_S2 * dt_s;
                            if (g_current_drive_speed_mm_s < 0.0f) { g_current_drive_speed_mm_s = 0.0f; }
                        } else if (new_ramp_phase == DriveRampPhase::ACCEL) {
                            g_current_drive_speed_mm_s += ACCEL_MM_S2 * dt_s;
                            if (g_current_drive_speed_mm_s > g_drive_speed_mm_s) {
                                g_current_drive_speed_mm_s = g_drive_speed_mm_s;
                            }
                        }
                        // CRUISE: no speed change

                        g_drive_distance_traveled_mm += g_current_drive_speed_mm_s * dt_s;
                        if (g_drive_distance_traveled_mm > g_drive_segment_len_mm) {
                            g_drive_distance_traveled_mm = g_drive_segment_len_mm;
                        }
                        g_last_drive_tick_us = now_us;
                        g_cmd_left_mm_s = g_current_drive_speed_mm_s;
                        g_cmd_right_mm_s = g_current_drive_speed_mm_s;
                    }

                    commandWheelSpeeds(g_cmd_left_mm_s, g_cmd_right_mm_s, now_us);
                    stepperControlTick(now_us);

                    // Phase completion: DRIVING uses distance, TURNING/WAITING use time deadline.
                    const bool phase_done = (g_segment_phase == SegmentPhase::DRIVING)
                        ? (g_drive_distance_traveled_mm >= g_drive_segment_len_mm)
                        : timeReachedUs(now_us, g_phase_deadline_us);

                    if (phase_done) {
                        if (g_segment_phase == SegmentPhase::TURNING) {
                            g_estimated_heading_rad = g_target_heading_rad;
                            // Transition to DRIVE: init ramp from zero.
                            g_drive_distance_traveled_mm = 0.0f;
                            g_current_drive_speed_mm_s = 0.0f;
                            g_drive_ramp_phase = DriveRampPhase::ACCEL;
                            g_last_drive_tick_us = now_us;
                            g_phase_deadline_us = 0;
                            g_cmd_left_mm_s = 0.0f;
                            g_cmd_right_mm_s = 0.0f;
                            g_segment_phase = SegmentPhase::DRIVING;
                            {
                                const float d_accel = (g_drive_speed_mm_s * g_drive_speed_mm_s) / (2.0f * ACCEL_MM_S2);
                                const float d_decel = (g_drive_speed_mm_s * g_drive_speed_mm_s) / (2.0f * DECEL_MM_S2);
                                LOG_WARN("Motion", "[ramp] TURN done -> DRIVE seg=%d len=%.1fmm target=%.1fmm/s d_accel=%.1fmm d_decel=%.1fmm",
                                         g_segment_index, g_drive_segment_len_mm,
                                         g_drive_speed_mm_s, d_accel, d_decel);
                            }
                        } else if (g_segment_phase == SegmentPhase::DRIVING) {
                            LOG_WARN("Motion", "[ramp] Segment %d complete: traveled=%.1fmm seg_len=%.1fmm final_spd=%.1fmm/s",
                                     g_segment_index, g_drive_distance_traveled_mm,
                                     g_drive_segment_len_mm, g_current_drive_speed_mm_s);
                            ++g_segment_index;
                            if (!startWaypointWaitIfNeeded(now_us) && !startNextSegment(now_us)) {
                                stopMotors();
                                g_segment_phase = SegmentPhase::IDLE;
                                g_state = MotionState::COMPLETED;
                            }
                        } else if (g_segment_phase == SegmentPhase::WAITING_POINT) {
                            if (!startNextSegment(now_us)) {
                                stopMotors();
                                g_segment_phase = SegmentPhase::IDLE;
                                g_state = MotionState::COMPLETED;
                            }
                        }
                    }
                }
                break;
            }
        case MotionState::PAUSED_OBSTACLE:
            {
                stopMotors();
                if (!g_blocked_by_obstacle) {
                    if (g_pause_started_us != 0) {
                        const uint32_t pause_duration_us = now_us - g_pause_started_us;
                        LOG_WARN("Motion", "[obstacle] Resuming: paused=%lums traveled=%.1fmm rem=%.1fmm — re-accel from 0",
                                 (unsigned long)(pause_duration_us / 1000),
                                 g_drive_distance_traveled_mm,
                                 g_drive_segment_len_mm - g_drive_distance_traveled_mm);
                        // Re-accelerate from zero after stop (motors were cut).
                        g_current_drive_speed_mm_s = 0.0f;
                        g_drive_ramp_phase = DriveRampPhase::ACCEL;
                        g_last_drive_tick_us = now_us;  // prevent dt spike
                        g_pause_started_us = 0;
                    }
                    g_state = MotionState::RUNNING;
                }
                break;
            }
        case MotionState::COMPLETED:
            {
                enforceHardMotorStop();
                analogWrite(ACTUATOR_PIN, 100);
                break;
            }

        case MotionState::FAULT:
            stopMotors();
            break;
    }

    applyLedPolicy();
    if (g_state != MotionState::COMPLETED)
        publishTelemetry(now_ms, now_us, distance_mm);
}

MotionState motionGetState() {
    return g_state;
}

Team motionGetTeam() {
    return g_candidate_team;
}

