#include <Arduino.h>
#include <math.h>

#include "generated_trajectory.h"
#include "led_status.h"
#include "motion_config.h"
#include "motion.h"
#include "stepper_control.h"

namespace {

using namespace MotionConfig;

enum class SegmentPhase : uint8_t {
    IDLE = 0,
    TURNING,
    DRIVING,
};

struct DebouncedInput {
    bool stable_value;
    bool raw_last;
    uint32_t last_change_ms;
};

MotionState g_state = MotionState::WAITING_TIRETTE;
Team g_candidate_team = Team::BLUE;
Team g_latched_team = Team::BLUE;
bool g_team_latched = false;
bool g_obstacle_blocked = false;
uint8_t g_us_invalid_streak = 0;
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
Waypoint g_working_trajectory[TRAJECTORY_POINTS_COUNT];

DebouncedInput g_team_input{};
DebouncedInput g_tirette_input{};

bool timeReachedUs(uint32_t now_us, uint32_t deadline_us) {
    return static_cast<int32_t>(now_us - deadline_us) >= 0;
}

void applyLedPolicy() {
    ledStatusApply(g_state, g_latched_team);
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

void buildWorkingTrajectory(Team latched_team) {
    for (int i = 0; i < TRAJECTORY_POINTS_COUNT; ++i) {
        Waypoint wp = EXPERIMENT_TRAJECTORY[i];
        if (latched_team == Team::YELLOW) {
            wp.x = MAP_WIDTH_MM - wp.x;
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

bool startNextSegment(uint32_t now_us) {
    while (g_segment_index < (TRAJECTORY_POINTS_COUNT - 1)) {
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
            const float turn_omega_rad_s = (2.0f * TURN_WHEEL_SPEED_MM_S) / TRACK_WIDTH_MM;
            const float turn_duration_s = abs_delta / turn_omega_rad_s;
            g_phase_deadline_us = now_us + static_cast<uint32_t>(turn_duration_s * 1000000.0f);

            if (delta_heading > 0.0f) {
                g_cmd_left_mm_s = -TURN_WHEEL_SPEED_MM_S;
                g_cmd_right_mm_s = TURN_WHEEL_SPEED_MM_S;
            } else {
                g_cmd_left_mm_s = TURN_WHEEL_SPEED_MM_S;
                g_cmd_right_mm_s = -TURN_WHEEL_SPEED_MM_S;
            }
            g_segment_phase = SegmentPhase::TURNING;
        } else {
            g_estimated_heading_rad = g_target_heading_rad;
            g_phase_deadline_us = now_us + g_pending_drive_duration_us;
            g_cmd_left_mm_s = g_drive_speed_mm_s;
            g_cmd_right_mm_s = g_drive_speed_mm_s;
            g_segment_phase = SegmentPhase::DRIVING;
        }
        return true;
    }
    return false;
}

void resetRunProgress() {
    g_segment_index = 0;
    g_phase_deadline_us = 0;
    g_pending_drive_duration_us = 0;
    g_pause_started_us = 0;
    g_estimated_heading_rad = 0.0f;
    g_target_heading_rad = 0.0f;
    g_cmd_left_mm_s = 0.0f;
    g_cmd_right_mm_s = 0.0f;
    g_segment_phase = SegmentPhase::IDLE;
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
    return (static_cast<float>(pulse_us) / 58.0f) * 10.0f;
}

void publishTelemetry(uint32_t now_ms, float distance_mm) {
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
    }

    Serial.printf("[motion] state=%s phase=%s team=%s seg=%d dist_mm=%.1f obstacle=%d us_bad=%u t_phase_ms=%lu\n",
                  state,
                  phase,
                  g_latched_team == Team::YELLOW ? "YELLOW" : "BLUE",
                  g_segment_index,
                  distance_mm,
                  g_obstacle_blocked ? 1 : 0,
                  static_cast<unsigned>(g_us_invalid_streak),
                  static_cast<unsigned long>(g_phase_deadline_us / 1000));
}

void stopMotors() {
    stepperControlStop();
}

} // namespace

void motionInit() {
    pinMode(TEAM_SWITCH_PIN, TEAM_SWITCH_PULLUP ? INPUT_PULLUP : INPUT);
    pinMode(TIRETTE_PIN, TIRETTE_PULLUP ? INPUT_PULLUP : INPUT);
    pinMode(US_TRIG_PIN, OUTPUT);
    pinMode(US_ECHO_PIN, INPUT);
    ledStatusInit();
    stepperControlInit();

    const bool team_raw = readDigitalActive(TEAM_SWITCH_PIN, TEAM_SWITCH_PULLUP);
    g_team_input = DebouncedInput{team_raw, team_raw, millis()};

    const bool tirette_raw = readDigitalActive(TIRETTE_PIN, TIRETTE_PULLUP);
    g_tirette_input = DebouncedInput{tirette_raw, tirette_raw, millis()};

    g_candidate_team = team_raw ? Team::YELLOW : Team::BLUE;
    g_latched_team = Team::BLUE;
    g_team_latched = false;
    g_state = MotionState::WAITING_TIRETTE;
    g_obstacle_blocked = false;
    g_us_invalid_streak = 0;
    resetRunProgress();
    g_start_deadline_us = 0;
    g_last_telemetry_ms = 0;

    stopMotors();
    applyLedPolicy();
}

void motionTick(uint32_t now_us) {
    const uint32_t now_ms = millis();

    const bool team_raw = readDigitalActive(TEAM_SWITCH_PIN, TEAM_SWITCH_PULLUP);
    const bool team_changed = updateDebounced(g_team_input, team_raw, now_ms, TEAM_DEBOUNCE_MS);
    if (!g_team_latched && team_changed) {
        g_candidate_team = g_team_input.stable_value ? Team::YELLOW : Team::BLUE;
    }

    const bool tirette_raw = readDigitalActive(TIRETTE_PIN, TIRETTE_PULLUP);
    const bool tirette_changed = updateDebounced(g_tirette_input, tirette_raw, now_ms, TIRETTE_DEBOUNCE_MS);
    const bool tirette_start_edge = tirette_changed && g_tirette_input.stable_value;

    float distance_mm = -1.0f;
    if (g_state == MotionState::RUNNING || g_state == MotionState::PAUSED_OBSTACLE) {
        distance_mm = readUltrasonicDistanceMm();
        if (distance_mm > 0.0f) {
            g_us_invalid_streak = 0;
            if (distance_mm <= OBSTACLE_STOP_MM) {
                g_obstacle_blocked = true;
            } else if (distance_mm >= OBSTACLE_RESUME_MM) {
                g_obstacle_blocked = false;
            }
        } else {
            if (g_us_invalid_streak < 255) {
                ++g_us_invalid_streak;
            }
            if (g_us_invalid_streak >= MAX_US_INVALID_BEFORE_BLOCK) {
                // Conservative behavior: hold robot paused when sensor data is stale.
                g_obstacle_blocked = true;
            }
        }
    }

    switch (g_state) {
        case MotionState::WAITING_TIRETTE:
            stopMotors();
            if (tirette_start_edge) {
                g_latched_team = g_candidate_team;
                g_team_latched = true;
                buildWorkingTrajectory(g_latched_team);
                resetRunProgress();
                g_start_deadline_us = now_us + static_cast<uint32_t>(START_AFTER_DELAY_S * 1000000.0f);
                g_state = MotionState::START_DELAY;
            }
            break;

        case MotionState::START_DELAY:
            stopMotors();
            if (timeReachedUs(now_us, g_start_deadline_us)) {
                if (!startNextSegment(now_us)) {
                    g_state = MotionState::COMPLETED;
                    break;
                }
                g_state = MotionState::RUNNING;
            }
            break;

        case MotionState::RUNNING:
            if (g_obstacle_blocked) {
                stopMotors();
                g_pause_started_us = now_us;
                g_state = MotionState::PAUSED_OBSTACLE;
            } else {
                commandWheelSpeeds(g_cmd_left_mm_s, g_cmd_right_mm_s, now_us);
                stepperControlTick(now_us);

                if (timeReachedUs(now_us, g_phase_deadline_us)) {
                    if (g_segment_phase == SegmentPhase::TURNING) {
                        g_estimated_heading_rad = g_target_heading_rad;
                        g_phase_deadline_us = now_us + g_pending_drive_duration_us;
                        g_cmd_left_mm_s = g_drive_speed_mm_s;
                        g_cmd_right_mm_s = g_drive_speed_mm_s;
                        g_segment_phase = SegmentPhase::DRIVING;
                    } else if (g_segment_phase == SegmentPhase::DRIVING) {
                        ++g_segment_index;
                        if (!startNextSegment(now_us)) {
                            stopMotors();
                            g_segment_phase = SegmentPhase::IDLE;
                            g_state = MotionState::COMPLETED;
                        }
                    }
                }
            }
            break;

        case MotionState::PAUSED_OBSTACLE:
            stopMotors();
            if (!g_obstacle_blocked) {
                if (g_pause_started_us != 0) {
                    g_phase_deadline_us += (now_us - g_pause_started_us);
                    g_pause_started_us = 0;
                }
                g_state = MotionState::RUNNING;
            }
            break;

        case MotionState::COMPLETED:
            stopMotors();
            break;

        case MotionState::FAULT:
            stopMotors();
            break;
    }

    applyLedPolicy();
    publishTelemetry(now_ms, distance_mm);
}

MotionState motionGetState() {
    return g_state;
}

Team motionGetLatchedTeam() {
    return g_latched_team;
}

