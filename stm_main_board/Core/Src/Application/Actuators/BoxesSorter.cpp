#include "Application/Actuators/BoxesSorter.h"

#include "Application/SCServosApp.h"
#include "Util/logging.h"


void BoxesSorter::prepareTopPusher()
{
    moveTopPusherToPosition(TOP_PUSHER_SERVO_POSITION_READY);
}

void BoxesSorter::grabAndSort2BoxesFromLift()
{
    if (teamColor == com_types::TeamColor::UNKNOWN)
    {
        LOG_ERROR("act", "Team color has not been set when trying to sort colors");
        return; // TODO put back
    }

    // prepareTopPusher() must have been called beforehand because here, lift shall be up with 2 boxes ready
    moveTopPusherToPosition(TOP_PUSHER_SERVO_POSITION_COLOR_SENSOR_FIRST_BOX);
    com_types::TeamColor firstBoxColor = colorSensor.detectColor();
    moveTopPusherToPosition(TOP_PUSHER_SERVO_POSITION_COLOR_SENSOR_FIRST_BOX + BOX_WIDTH);
    com_types::TeamColor secondBoxColor = colorSensor.detectColor();

    if (IS_COLOR_SENSOR_UNDERNEATH_BOXES)
    {
        firstBoxColor = colorSensor.inverseColors(firstBoxColor);
        secondBoxColor = colorSensor.inverseColors(secondBoxColor);
    }

    if (firstBoxColor == com_types::TeamColor::UNKNOWN or secondBoxColor == com_types::TeamColor::UNKNOWN)
    {
        LOG_ERROR("act", "Could not find the color of a box: 1=%d, 2=%d", firstBoxColor, secondBoxColor);
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
    moveBottomPusherToPosition(BOTTOM_PUSHER_SERVO_POSITION_READY);
}

void BoxesSorter::openExitRamp()
{
    devices::scs_servos::set_angle_async(EXIT_RAMP_SERVO_ID, EXIT_RAMP_SERVO_RELEASED, 300);
}

void BoxesSorter::initialize()
{
    // Move top pusher out
    devices::scs_servos::homingByStall(TOP_PUSHER_SERVO_ID, 300, 300);
    // Move bottom pusher in
    devices::scs_servos::homingByStall(BOTTOM_PUSHER_SERVO_ID, 300, 300);
    // Close first hole trapdoor
    _closeTrapdoor();
    // Close exit ramp servo
    devices::scs_servos::set_angle_async(EXIT_RAMP_SERVO_ID, EXIT_RAMP_SERVO_IDLE, 300);
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
    // 1. Define your mechanical constant (Calibration needed!)
    // Example: At speed 200, the pusher moves at 30mm/s
    static constexpr float MM_PER_SEC_AT_BASE_SPEED = 30.0f; // TODO TO TEST

    const float distanceToMove = target - currentPosition;
    if (abs(distanceToMove) < 0.1f) return; // Already there

    // 3. Calculate direction and duration
    const int direction = (distanceToMove > 0) ? 1 : -1;
    const float durationSeconds = abs(distanceToMove) / MM_PER_SEC_AT_BASE_SPEED;
    const uint32_t durationMs = static_cast<uint32_t>(durationSeconds * 1000.0f);

    // 4. Execute movement
    LOG_INFO("sorter", "Moving pusher %d to %.1f mm (Duration: %lu ms)", speed, target, durationMs);

    // Start motor
    devices::scs_servos::set_speed(servoID, speed * direction);

    // Wait for the calculated time
    osDelay(durationMs);

    // Stop motor
    devices::scs_servos::set_speed(servoID, 0);
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
