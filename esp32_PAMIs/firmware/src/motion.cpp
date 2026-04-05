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
bool g_blocked_by_obstacle = false;
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

DebouncedInput g_tirette_input{};

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
    for (int i = 0; i < TRAJECTORY_POINTS_COUNT; ++i) {
        Waypoint wp = EXPERIMENT_TRAJECTORY[i];
        if (team == Team::YELLOW) {
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

bool startNextSegment(const uint32_t now_us) {
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
            constexpr float turn_omega_rad_s = (2.0f * TURN_WHEEL_SPEED_MM_S) / ENTRAXE_MM;
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
        LOG_WARN("Motion", "Starting segment %d -> %d: len=%.1fmm turn=%.1fdeg drive=%.1fs",
                 g_segment_index, g_segment_index + 1, segment_len_mm, delta_heading*180.0/M_PI, drive_duration_s);
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
    return (static_cast<float>(pulse_us) / 58.3f) * 10.0f;
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

    Serial.printf("[motion] state=%s phase=%s team=%s seg=%d dist_mm=%.1f obstacle=%d t_phase_ms=%lu\n",
                  state,
                  phase,
                  g_candidate_team == Team::YELLOW ? "YELLOW" : "BLUE",
                  g_segment_index,
                  distance_mm,
                  g_blocked_by_obstacle ? 1 : 0,
                  static_cast<unsigned long>(g_phase_deadline_us / 1000));
}

void stopMotors() {
    stepperControlStop();
}

} // namespace

void motionInit() {
    LOG_INFO("Motion", "Initializing system...");
    pinMode(TEAM_SWITCH_PIN, TEAM_SWITCH_PULLUP ? INPUT_PULLUP : INPUT);
    pinMode(TIRETTE_PIN, TIRETTE_PULLUP ? INPUT_PULLUP : INPUT);
    pinMode(US_TRIG_PIN, OUTPUT);
    pinMode(US_ECHO_PIN, INPUT);
    LOG_INFO("Motion", "Initialized pins...");
    ledStatusInit();
    LOG_INFO("Motion", "Initialized status LED...");
    stepperControlInit();
    stopMotors();
    LOG_INFO("Motion", "Initialized stepperControl...");

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
        //     bool tirette = digitalRead(TIRETTE_PIN);
        //     LOG_INFO("tirette", "tirette %d team %d", tirette, team);
        //     delay(500);
        // }
    }

    const bool team_raw = readDigitalActive(TEAM_SWITCH_PIN, TEAM_SWITCH_PULLUP);
    const bool tirette_raw = readDigitalActive(TIRETTE_PIN, TIRETTE_PULLUP);
    g_tirette_input = DebouncedInput{tirette_raw, tirette_raw, millis()};

    g_candidate_team = team_raw ? Team::YELLOW : Team::BLUE;
    g_state = MotionState::WAITING_TIRETTE;
    g_blocked_by_obstacle = false;
    resetRunProgress();
    g_start_deadline_us = 0;
    g_last_telemetry_ms = 0;

    applyLedPolicy();
}

void motionTick(uint32_t now_us) {
    const uint32_t now_ms = millis();

    const bool tirette_raw = readDigitalActive(TIRETTE_PIN, TIRETTE_PULLUP);
    const bool tirette_changed = updateDebounced(g_tirette_input, tirette_raw, now_ms, TIRETTE_DEBOUNCE_MS);
    const bool tirette_start_edge = tirette_changed && g_tirette_input.stable_value;

    float distance_mm = -1.0f;
    if (g_state == MotionState::RUNNING || g_state == MotionState::PAUSED_OBSTACLE) {
        distance_mm = readUltrasonicDistanceMm();
        if (distance_mm > 0.0f) {
            if (distance_mm <= OBSTACLE_STOP_MM) {
                g_blocked_by_obstacle = true;
            } else if (distance_mm >= OBSTACLE_RESUME_MM) {
                g_blocked_by_obstacle = false;
            }
        }
    }

    switch (g_state) {
        case MotionState::WAITING_TIRETTE:
            {
                const bool team_raw = readDigitalActive(TEAM_SWITCH_PIN, TEAM_SWITCH_PULLUP);
                g_candidate_team = team_raw ? Team::YELLOW : Team::BLUE;

                stopMotors();
                if (tirette_start_edge) {
                    buildWorkingTrajectory(g_candidate_team);
                    resetRunProgress();
                    g_start_deadline_us = now_us + static_cast<uint32_t>(START_AFTER_DELAY_S * 1000000.0f);
                    g_state = MotionState::START_DELAY;
                }
                break;
            }
        case MotionState::START_DELAY:
            {
                stopMotors();
                if (timeReachedUs(now_us, g_start_deadline_us)) {
                    if (!startNextSegment(now_us)) {
                        g_state = MotionState::COMPLETED;
                        break;
                    }
                    g_state = MotionState::RUNNING;
                }
                break;
            }
        case MotionState::RUNNING:
            {
                if (g_blocked_by_obstacle) {
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
            }
        case MotionState::PAUSED_OBSTACLE:
            {
                stopMotors();
                if (!g_blocked_by_obstacle) {
                    if (g_pause_started_us != 0) {
                        g_phase_deadline_us += (now_us - g_pause_started_us);
                        g_pause_started_us = 0;
                    }
                    g_state = MotionState::RUNNING;
                }
                break;
            }
        case MotionState::COMPLETED:
            {
                stopMotors();
                break;
            }

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

Team motionGetTeam() {
    return g_candidate_team;
}

