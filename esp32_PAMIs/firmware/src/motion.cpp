#include <Arduino.h>
#include <math.h>

#include "generated_trajectory.h"
#include "config.h"
#include "motion.h"
#include "inputs.h"
#include "stepper_control.h"
#include "logging.h"

namespace {

using namespace Config;

enum class SegmentPhase : uint8_t {
    IDLE = 0,
    TURNING,
    DRIVING,
    WAITING_POINT,
};

MotionState g_state = MotionState::WAITING_TIRETTE;
Team g_candidate_team = Team::BLUE;
bool g_blocked_by_obstacle = false;
int g_segment_index = 0;
uint32_t g_start_deadline_us = 0;
uint32_t g_last_telemetry_ms = 0;
uint32_t g_phase_deadline_us = 0;
float g_estimated_heading_rad = 0.0f;
float g_target_heading_rad = 0.0f;
float g_drive_speed_mm_s = GLOBAL_SPEED_MM_S;
float g_drive_segment_len_mm = 0.0f;       // length of the current drive segment
SegmentPhase g_segment_phase = SegmentPhase::IDLE;

Waypoint g_working_trajectory[TRAJECTORY_POINTS_COUNT];
bool g_match_timeout_armed = false;
bool g_match_timeout_triggered = false;
uint32_t g_match_timeout_deadline_us = 0;

constexpr uint32_t kMatchHardStopUs = 100000000UL; // 100 s after pull-cord start.

constexpr int kGeneratedTrajectoryPoints = static_cast<int>(sizeof(EXPERIMENT_TRAJECTORY) / sizeof(EXPERIMENT_TRAJECTORY[0]));

int activeTrajectoryPoints() {
    return (kGeneratedTrajectoryPoints < TRAJECTORY_POINTS_COUNT) ? kGeneratedTrajectoryPoints : TRAJECTORY_POINTS_COUNT;
}

bool timeReachedUs(uint32_t now_us, uint32_t deadline_us) {
    return static_cast<int32_t>(now_us - deadline_us) >= 0;
}

void buildWorkingTrajectory(Team team) {
    const int points = activeTrajectoryPoints();
    for (int i = 0; i < points; ++i) {
        Waypoint wp = EXPERIMENT_TRAJECTORY[i];
        if (team == Team::BLUE) {
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

        if (abs_delta >= MIN_TURN_RAD) {
            // TURNING phase
            // For now, use the same ACCEL profile but cap angular speed
            stepperControlSetProfile(TURN_WHEEL_SPEED_MM_S, ACCEL_MM_S2, DECEL_MM_S2);
            stepperControlTurn(delta_heading);

            g_drive_segment_len_mm = segment_len_mm;  // stored for DRIVE init after turn
            g_segment_phase = SegmentPhase::TURNING;
        } else {
            // No turn needed — start DRIVING directly.
            g_estimated_heading_rad = g_target_heading_rad;
            g_drive_segment_len_mm = segment_len_mm;
            g_segment_phase = SegmentPhase::DRIVING;

            stepperControlSetProfile(g_drive_speed_mm_s, ACCEL_MM_S2, DECEL_MM_S2);
            stepperControlDrive(segment_len_mm);
        }
        
        LOG_WARN("Motion", "[seg] %d->%d len=%.1fmm turn=%.1fdeg target=%.1fmm/s",
                     g_segment_index, g_segment_index + 1, segment_len_mm,
                     delta_heading * 180.0 / M_PI, g_drive_speed_mm_s);
        
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
    stepperControlDecelerateStop();
    g_segment_phase = SegmentPhase::WAITING_POINT;
    return true;
}

void resetRunProgress() {
    g_segment_index = 0;
    g_phase_deadline_us = 0;
    g_target_heading_rad = 0.0f;
    g_segment_phase = SegmentPhase::IDLE;
    g_drive_segment_len_mm = 0.0f;

    // Set initial heading from the first waypoint orientation.
    const int points = activeTrajectoryPoints();
    if (points >= 1) {
        g_estimated_heading_rad = g_working_trajectory[0].headingDeg * static_cast<float>(M_PI) / 180.0f;
    } else {
        g_estimated_heading_rad = 0.0f;
    }
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

    const int points = activeTrajectoryPoints();
    const int max_segment_index = (points > 0) ? (points - 1) : 0;
    const int segment_from = (g_segment_index < max_segment_index) ? g_segment_index : max_segment_index;
    const int segment_to = (segment_from < max_segment_index) ? (segment_from + 1) : segment_from;

    float traveled_mm = stepperControlTraveledMm();
    float seg_rem_mm = g_drive_segment_len_mm - traveled_mm;
    if (seg_rem_mm < 0.0f) seg_rem_mm = 0.0f;

    Serial.printf("[motion] state=%s phase=%s team=%s seg=%d->%d traveled_mm=%.1f seg_rem_mm=%.1f obstacle=%d us_mm=%.1f\n",
                  state,
                  phase,
                  g_candidate_team == Team::YELLOW ? "YELLOW" : "BLUE",
                  segment_from,
                  segment_to,
                  traveled_mm,
                  seg_rem_mm,
                  g_blocked_by_obstacle ? 1 : 0,
                  distance_mm);
}

void printStepperDiagnostics() {
    const float wheel_circumference_mm = static_cast<float>(PI) * WHEEL_DIAMETER_MM;
    const float steps_per_wheel_turn = MOTOR_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO;
    const float steps_per_mm = steps_per_wheel_turn / wheel_circumference_mm;

    LOG_WARN("Stepper", "=== Diagnostics ===");
    LOG_WARN("Stepper", "steps_per_mm=%.2f", steps_per_mm);
    LOG_WARN("Stepper", "ACCEL=%.0fmm/s²  DECEL=%.0fmm/s²  target=%.0fmm/s", ACCEL_MM_S2, DECEL_MM_S2, GLOBAL_SPEED_MM_S);
}

} // namespace

void motionInit() {
    LOG_INFO("Motion", "Initializing system...");
    pinMode(US_TRIG_PIN, OUTPUT);
    pinMode(US_ECHO_PIN, INPUT);

    stepperControlInit();
    LOG_INFO("Motion", "Initialized stepperControl...");

    g_candidate_team = inputsReadTeam();
    g_state = MotionState::WAITING_TIRETTE;
    g_blocked_by_obstacle = false;
    g_match_timeout_armed = false;
    g_match_timeout_triggered = false;
    g_match_timeout_deadline_us = 0;
    resetRunProgress();
    g_start_deadline_us = 0;
    g_last_telemetry_ms = 0;

    if (kGeneratedTrajectoryPoints != TRAJECTORY_POINTS_COUNT) {
        LOG_WARN("Motion", "Trajectory count mismatch: declared=%d generated=%d (using %d)",
                 TRAJECTORY_POINTS_COUNT, kGeneratedTrajectoryPoints, activeTrajectoryPoints());
    }

    printStepperDiagnostics();
}

void motionTick(uint32_t now_us) {
    //log state
    // LOG_INFO("Motion", "Tick: state=%d team=%d seg=%d phase=%d", static_cast<int>(g_state), static_cast<int>(g_candidate_team), g_segment_index, static_cast<int>(g_segment_phase));
    const uint32_t now_ms = millis();

    if (g_match_timeout_armed && !g_match_timeout_triggered && timeReachedUs(now_us, g_match_timeout_deadline_us)) {
        g_match_timeout_triggered = true;
        g_state = MotionState::COMPLETED;
        LOG_WARN("Motion", "Hard stop timeout reached (100s after pull-cord). Motors disabled permanently.");
    }

    if (g_match_timeout_triggered) {
        stepperControlDisable();
        return;
    }

    bool tirette_active = false; // TODO test
    inputsTiretteIsActive();
    if (g_state == MotionState::WAITING_TIRETTE) {
        tirette_active = inputsTiretteIsActive();
    }

    float distance_mm = -1.0f;
    distance_mm = readUltrasonicDistanceMm();
    if (distance_mm > 0.1f) {
        if (distance_mm <= OBSTACLE_STOP_MM) {
            g_blocked_by_obstacle = true;
        } else if (distance_mm >= OBSTACLE_RESUME_MM) {
            g_blocked_by_obstacle = false;
        }
    }

    switch (g_state) {
        case MotionState::WAITING_TIRETTE:
            {
                g_candidate_team = inputsReadTeam();
                stepperControlDisable(); 
                if (tirette_active) {
                    buildWorkingTrajectory(g_candidate_team);
                    resetRunProgress();
                    g_start_deadline_us = now_us + static_cast<uint32_t>(DELAY_AFTER_PULL_CORD_S * 1000000.0f);
                    g_match_timeout_armed = true;
                    g_match_timeout_deadline_us = now_us + kMatchHardStopUs;
                    g_state = MotionState::START_DELAY;
                    LOG_WARN("Motion", "Tirette pulled! Starting delay... team=%s start_in=%.1fs",
                             g_candidate_team == Team::YELLOW ? "YELLOW" : "BLUE",
                             static_cast<float>(g_start_deadline_us - now_us) / 1000000.0f);
                }
                break;
            }
        case MotionState::START_DELAY:
            {
                stepperControlDisable();
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
                    stepperControlDecelerateStop();
                    LOG_WARN("Motion", "[obstacle] Pausing mid-segment: traveled=%.1fmm rem=%.1fmm",
                             stepperControlTraveledMm(),
                             g_drive_segment_len_mm - stepperControlTraveledMm());
                    g_state = MotionState::PAUSED_OBSTACLE;
                    // disable ENABLE PIN
                    digitalWrite(ENABLE_MOTORS, HIGH); // Enable hold torque
                } else {
                    digitalWrite(ENABLE_MOTORS, LOW); // Disable hold torque
                    // Phase completion: DRIVING/TURNING use stepperControlIsRunning(), WAITING uses time deadline.
                    bool phase_done = false;
                    if (g_segment_phase == SegmentPhase::DRIVING || g_segment_phase == SegmentPhase::TURNING) {
                        phase_done = !stepperControlIsRunning();
                    } else if (g_segment_phase == SegmentPhase::WAITING_POINT) {
                        phase_done = timeReachedUs(now_us, g_phase_deadline_us);
                    }

                    if (phase_done) {
                        if (g_segment_phase == SegmentPhase::TURNING) {
                            g_estimated_heading_rad = g_target_heading_rad;
                            
                            // Transition to DRIVE
                            g_segment_phase = SegmentPhase::DRIVING;
                            stepperControlSetProfile(g_drive_speed_mm_s, ACCEL_MM_S2, DECEL_MM_S2);
                            stepperControlDrive(g_drive_segment_len_mm);
                            
                            LOG_WARN("Motion", "TURN done -> DRIVE seg=%d len=%.1fmm target=%.1fmm/s",
                                     g_segment_index, g_drive_segment_len_mm, g_drive_speed_mm_s);
                        } else if (g_segment_phase == SegmentPhase::DRIVING) {
                            LOG_WARN("Motion", "Segment %d complete: traveled=%.1fmm seg_len=%.1fmm",
                                     g_segment_index, stepperControlTraveledMm(), g_drive_segment_len_mm);
                            ++g_segment_index;
                            if (!startWaypointWaitIfNeeded(now_us) && !startNextSegment(now_us)) {
                                stepperControlDecelerateStop();
                                g_segment_phase = SegmentPhase::IDLE;
                                g_state = MotionState::COMPLETED;
                            }
                        } else if (g_segment_phase == SegmentPhase::WAITING_POINT) {
                            if (!startNextSegment(now_us)) {
                                stepperControlDecelerateStop();
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
                if (!g_blocked_by_obstacle) {
                    LOG_WARN("Motion", "[obstacle] Resuming segment %d", g_segment_index);
                    stepperControlSetProfile(g_drive_speed_mm_s, ACCEL_MM_S2, DECEL_MM_S2);
                    stepperControlResume();
                    g_state = MotionState::RUNNING;
                    digitalWrite(ENABLE_MOTORS, LOW); // Disable hold torque
                }
                break;
            }
        case MotionState::COMPLETED:
            stepperControlDisable();
            break;

        case MotionState::FAULT:
            stepperControlDecelerateStop();
            break;
    }

    if (g_state != MotionState::COMPLETED) {
        publishTelemetry(now_ms, now_us, distance_mm);
    }
}

MotionState motionGetState() {
    return g_state;
}

Team motionGetTeam() {
    return g_candidate_team;
}
