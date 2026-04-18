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
    devices::scs_servos::homingByEndSwitch(BOTTOM_PUSHER_SERVO_ID, -BOTTOM_PUSHER_SERVO_SPEED, BOTTOM_END_SWITCH_GPIO_Port, BOTTOM_END_SWITCH_GPIO_Pin);
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
    devices::scs_servos::homingByEndSwitch(BOTTOM_PUSHER_SERVO_ID, -500, BOTTOM_END_SWITCH_GPIO_Port, BOTTOM_END_SWITCH_GPIO_Pin);
    bottomPusherPosition = 0;
    osDelay(1000);
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

    float totalStepsMoved = 0.0f;
    int prevPos = startPos;
    constexpr int TIMEOUT_MS = 10000;
    int elapsedMs = 0;
    int readErrors = 0;

    while (totalStepsMoved < stepsNeeded)
    {
        osDelay(5); // poll at ~200 Hz
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

        // Compute delta with wrap-around handling
        int delta = nowPos - prevPos;

        if (delta > ENCODER_RANGE / 2)
            delta -= ENCODER_RANGE;
        else if (delta < -ENCODER_RANGE / 2)
            delta += ENCODER_RANGE;

        totalStepsMoved += fabsf(static_cast<float>(delta));

        LOG_INFO_THROTTLE("sorter", 100, "Servo %d: nowPos=%d, delta=%d, totalMoved=%.0f/%.0f",
                          servoID, nowPos, delta, totalStepsMoved, stepsNeeded);

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
