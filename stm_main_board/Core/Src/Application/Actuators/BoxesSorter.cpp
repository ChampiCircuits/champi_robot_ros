#include "Application/Actuators/BoxesSorter.h"

#include "Application/SCServosApp.h"
#include "Util/logging.h"
#include <cmath>


void BoxesSorter::prepareTopPusher()
{
    moveTopPusherToPosition(TOP_PUSHER_SERVO_POSITION_READY);
}

void BoxesSorter::grabAndSort2BoxesFromLift()
{
    if (teamColor == com_types::TeamColor::UNKNOWN)
    {
        LOG_ERROR("act", "Team color has not been set when trying to sort colors");
        return;
    }

    // prepareTopPusher() must have been called beforehand because here, lift shall be up with 2 boxes ready
    moveTopPusherToPosition(TOP_PUSHER_SERVO_POSITION_COLOR_SENSOR_FIRST_BOX);
    com_types::TeamColor firstBoxColor = colorSensor.detectColor();
    moveTopPusherToPosition(TOP_PUSHER_SERVO_POSITION_COLOR_SENSOR_FIRST_BOX + BOX_WIDTH);
    com_types::TeamColor secondBoxColor = colorSensor.detectColor();

    if (IS_COLOR_SENSOR_UNDERNEATH_BOXES)
    {
        firstBoxColor = ColorSensorTCS34725::inverseColors(firstBoxColor);
        secondBoxColor = ColorSensorTCS34725::inverseColors(secondBoxColor);
    }

    if (firstBoxColor == com_types::TeamColor::UNKNOWN or secondBoxColor == com_types::TeamColor::UNKNOWN)
    {
        LOG_ERROR("act", "Could not find the color of a box: 1=%s, 2=%s", to_c_str(firstBoxColor), to_c_str(secondBoxColor));
        return;
    }

    const com_types::TeamColor boxesColor[2] = {firstBoxColor, secondBoxColor};
    for (int i = 0; i < 2; i++)
    {
        const bool isColorOK = (boxesColor[i] == teamColor);

        // 1. Set the trapdoor state
        if (isColorOK)  _openTrapdoor();
        else            _closeTrapdoor();

        // 2. Determine target hole base position
        const float holePos = isColorOK ? TOP_PUSHER_SERVO_POSITION_STRAIGHT_HOLE_FIRST_BOX
                               : TOP_PUSHER_SERVO_POSITION_TOBOGGAN_HOLE_FIRST_BOX;
        // 3. Move the pusher
        // (i * BOX_WIDTH) offsets the pusher because the 2nd box is further back
        moveTopPusherToPosition(holePos + (i * BOX_WIDTH));
    }

    // Return to init position after sorting
    moveTopPusherToPosition(TOP_PUSHER_SERVO_POSITION_READY);
}

void BoxesSorter::push2BoxesOut()
{
    moveBottomPusherToPosition(BOTTOM_PUSHER_SERVO_POSITION_OUT);
    devices::scs_servos::homingByEndSwitch(BOTTOM_PUSHER_SERVO_ID, -BOTTOM_PUSHER_SERVO_SPEED, BOTTOM_END_SWITCH_GPIO_Port, BOTTOM_END_SWITCH_GPIO_Pin, false);
    bottomPusherPosition = 0;
    osDelay(1000);
    moveBottomPusherToPosition(BOTTOM_PUSHER_SERVO_POSITION_READY); // si on fait pas ca, il faut écrire à la main la position
}

void BoxesSorter::openExitRamp()
{
    devices::scs_servos::set_angle_async(EXIT_RAMP_SERVO_ID, EXIT_RAMP_SERVO_RELEASED, 300);
}

void BoxesSorter::initialize()
{
    LOG_INFO("box_sorter", "initializing boxes sorter...");
    // Move top pusher out
    // devices::scs_servos::homingByEndSwitch(TOP_PUSHER_SERVO_ID, 500, BOTTOM_END_SWITCH_GPIO_Port, BOTTOM_END_SWITCH_GPIO_Pin);
    // topPusherPosition = 0;
    // moveBottomPusherToPosition(TOP_PUSHER_SERVO_POSITION_READY);
    // Move bottom pusher in
    devices::scs_servos::homingByEndSwitch(BOTTOM_PUSHER_SERVO_ID, -500, BOTTOM_END_SWITCH_GPIO_Port, BOTTOM_END_SWITCH_GPIO_Pin, true);
    bottomPusherPosition = 0;
    osDelay(500);
    // move pusher a bit more inside (end switch is too much inside)
    moveBottomPusherToPosition(BOTTOM_PUSHER_SERVO_POSITION_READY);


    // Close first hole trapdoor
    // _closeTrapdoor();
    // Close exit ramp servo
    // devices::scs_servos::set_angle_async(EXIT_RAMP_SERVO_ID, EXIT_RAMP_SERVO_IDLE, 300);
    LOG_INFO("box_sorter", "initialized boxes sorter...");
}

void BoxesSorter::setTeamColor(const com_types::TeamColor color)
{
    teamColor = color;
    LOG_INFO("act", "Received team color : %s", to_c_str(teamColor));
}

void BoxesSorter::_closeTrapdoor()
{
    devices::scs_servos::set_angle(TRAPDOOR_SERVO_ID, TRAPDOOR_SERVO_CLOSED, 300);
}

void BoxesSorter::_openTrapdoor()
{
    devices::scs_servos::set_angle(TRAPDOOR_SERVO_ID, TRAPDOOR_SERVO_OPEN, 300);
}

void BoxesSorter::_movePusherToPosition(int servoID, int speed, float target, float currentPosition)
{
    const float distanceToMove = target - currentPosition;
    if (fabsf(distanceToMove) < 0.1f) return; // Already there

    const int direction = (distanceToMove > 0) ? 1 : -1;
    const float stepsNeeded = fabsf(distanceToMove);

    // Read start position from encoder
    const int startPos = devices::scs_servos::read_position_raw(servoID);
    if (startPos < 0) {
        LOG_ERROR("sorter", "Failed to read position for servo %d", servoID);
        return;
    }

    LOG_INFO("sorter", "Moving servo %d: target=%.1f, current=%.1f, distance=%.1f, stepsNeeded=%.0f, dir=%d, startPos=%d",
             servoID, target, currentPosition, distanceToMove, stepsNeeded, direction, startPos);

    // Start motor
    devices::scs_servos::set_speed(servoID, speed * direction);

    // SCS15 encoder covers ~200° of 360° rotation (0-1023).
    // The remaining ~160° is a "dead zone" where the encoder saturates at 0 or ~1022.
    // During the dead zone the motor is still spinning, so we estimate steps from time.
    // Measured: ~1020 readable steps per revolution, dead zone ≈ DEAD_ZONE_STEPS.
    constexpr int DEAD_ZONE_NEAR_MIN = 5;          // encoder stuck near 0
    constexpr int DEAD_ZONE_NEAR_MAX = 1018;       // encoder stuck near 1023
    constexpr int DEAD_ZONE_STUCK_THRESHOLD_MS = 50; // if stuck this long, we're in dead zone
    constexpr float STEPS_PER_MS_AT_SPEED_500 = 1.7f; // calibrate: encoder steps per ms at speed 500 TODO

    float totalStepsMoved = 0.0f;
    int prevPos = startPos;
    constexpr int TIMEOUT_MS = 15000;
    constexpr int STALL_TIMEOUT_MS = 1000;
    int elapsedMs = 0;
    int stuckMs = 0;       // time encoder hasn't changed
    int stuckPos = -1;     // position where we got stuck
    bool inDeadZone = false;
    int readErrors = 0;

    // Speed factor for dead zone estimation
    float stepsPerMs = STEPS_PER_MS_AT_SPEED_500 * (static_cast<float>(speed) / 500.0f);

    while (totalStepsMoved < stepsNeeded)
    {
        osDelay(5);
        elapsedMs += 5;

        if (elapsedMs >= TIMEOUT_MS) {
            LOG_ERROR("sorter", "Timeout servo %d! Moved %.0f/%.0f steps", servoID, totalStepsMoved, stepsNeeded);
            break;
        }

        int nowPos = devices::scs_servos::read_position_raw(servoID);
        if (nowPos < 0) {
            readErrors++;
            if (readErrors > 100) {
                LOG_ERROR("sorter", "Too many read errors for servo %d, aborting", servoID);
                break;
            }
            continue;
        }
        readErrors = 0;

        int delta = nowPos - prevPos;
        int absDelta = abs(delta);

        bool isNearEdge = (nowPos <= DEAD_ZONE_NEAR_MIN || nowPos >= DEAD_ZONE_NEAR_MAX);

        if (absDelta <= 1 && isNearEdge) {
            // Encoder is stuck near 0 or 1022 — possibly in dead zone
            stuckMs += 5;

            if (stuckMs >= DEAD_ZONE_STUCK_THRESHOLD_MS && !inDeadZone) {
                inDeadZone = true;
                stuckPos = nowPos;
                LOG_INFO("sorter", "Servo %d: entered dead zone at pos %d (total=%.0f)", servoID, nowPos, totalStepsMoved);
            }

            if (inDeadZone) {
                // Estimate movement from time
                totalStepsMoved += stepsPerMs * 5.0f;
            }
        } else if (inDeadZone) {
            // Large jump while exiting dead zone — ignore the delta value (it's garbage)
            if (absDelta > 50) {
                // Encoder jumped to a new region after dead zone, don't count this delta
                LOG_INFO("sorter", "Servo %d: exiting dead zone, jump to %d (ignoring delta %d)", servoID, nowPos, delta);
            } else {
                // Small delta, we're back in readable range
                totalStepsMoved += static_cast<float>(absDelta);
            }
            inDeadZone = false;
            stuckMs = 0;
            LOG_INFO("sorter", "Servo %d: exited dead zone at pos %d (total=%.0f)", servoID, nowPos, totalStepsMoved);
        } else {
            // Normal tracking
            stuckMs = 0;

            if (absDelta > 0 && absDelta < 50) {
                totalStepsMoved += static_cast<float>(absDelta);
            } else if (absDelta >= 50) {
                // Large jump (garbage read or dead zone transition) — ignore
                LOG_INFO("sorter", "Servo %d: ignoring large delta %d (prev=%d, now=%d)", servoID, delta, prevPos, nowPos);
            }
            // absDelta == 0 with no edge: could be real stall
            if (absDelta == 0 && !isNearEdge) {
                stuckMs += 5;
                if (stuckMs >= STALL_TIMEOUT_MS) {
                    LOG_ERROR("sorter", "Servo %d stalled at pos %d! Moved %.0f/%.0f steps", servoID, nowPos, totalStepsMoved, stepsNeeded);
                    break;
                }
            }
        }

        LOG_INFO_THROTTLE("sorter", 1, "Servo %d: nowPos=%d, totalMoved=%.0f/%.0f, dz=%d",
                          servoID, nowPos, totalStepsMoved, stepsNeeded, inDeadZone ? 1 : 0);

        prevPos = nowPos;
    }

    // Stop motor (send multiple times to ensure it's received)
    for (int i = 0; i < 3; i++) {
        devices::scs_servos::set_speed(servoID, 0);
        osDelay(10);
    }

    int finalPos = devices::scs_servos::read_position_raw(servoID);
    LOG_INFO("sorter", "Servo %d stopped. Total steps: %.0f, finalPos=%d", servoID, totalStepsMoved, finalPos);
}

void BoxesSorter::moveTopPusherToPosition(float target)
{
    _movePusherToPosition(TOP_PUSHER_SERVO_ID, TOP_PUSHER_SERVO_SPEED, target, topPusherPosition);
    topPusherPosition = target;
}

void BoxesSorter::moveBottomPusherToPosition(float target)
{
    _movePusherToPosition(BOTTOM_PUSHER_SERVO_ID, BOTTOM_PUSHER_SERVO_SPEED, target, bottomPusherPosition);
    bottomPusherPosition = target;
}
