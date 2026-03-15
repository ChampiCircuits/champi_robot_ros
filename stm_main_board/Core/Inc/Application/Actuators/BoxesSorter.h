#ifndef STM_MAIN_BOARD_BOXESSORTER_H
#define STM_MAIN_BOARD_BOXESSORTER_H

#include "Application/Modbus/DataStructures.h"
#include "Devices/ColorSensorTCS34725.h"

/**
 * BOXES SORTER actuators :
 * - TOP PUSHER : 1 servo to bring the boxes 2 by 2 from the lift
 * - 1 COLOR SENSOR
 * - 2 holes : 1 for each color
 *      - straight hole
 *      - toboggan hole -> inverse color
 * -
 */

// TODO the STM SHALL RECEIVE the color of the team

class BoxesSorter
{
public:
    void prepareTopPusher();
    void grabAndSort2BoxesFromLift();
    void push2BoxesOut();
    void initialize();
    void setTeamColor(com_types::TeamColor color) { teamColor = color; }
    void moveTopPusherToPosition(float target);
    void moveBottomPusherToPosition(float target);

    int boxesInSorterCount = 0; // always an even number (clamp takes boxes 2 by 2)

private:
    static constexpr float BOX_WIDTH = 50.0; // mm --> pusher increment

    // TOP PUSHER SERVO (FREE ROTATION SERVO)
    // | TOBOGGAN - STRAIGHT HOLE - COLOR SENSOR - LIFT - READY POSITION |
    static constexpr int TOP_PUSHER_SERVO_ID = 2; // TODO
    static constexpr float TOP_PUSHER_SERVO_SPEED  = 200;  // [0,1023]
    static constexpr float TOP_PUSHER_SERVO_POSITION_READY  = 0;
    static constexpr float TOP_PUSHER_SERVO_POSITION_COLOR_SENSOR_FIRST_BOX  = 200;  // mm // TODO
    static constexpr float TOP_PUSHER_SERVO_POSITION_STRAIGHT_HOLE_FIRST_BOX  = 300; // mm // TODO
    static constexpr float TOP_PUSHER_SERVO_POSITION_TOBOGGAN_HOLE_FIRST_BOX  = 400; // mm // TODO
    float topPusherPosition = TOP_PUSHER_SERVO_POSITION_READY;

    // BOTTOM PUSHER SERVO (FREE ROTATION SERVO)
    // | OUT | TOBOGGAN - STRAIGHT HOLE - READY |
    static constexpr int BOTTOM_PUSHER_SERVO_ID = 3; // TODO
    static constexpr float BOTTOM_PUSHER_SERVO_SPEED  = 200;  // [0,1023]
    static constexpr float BOTTOM_PUSHER_SERVO_POSITION_READY  = 0;  // mm // TODO
    static constexpr float BOTTOM_PUSHER_SERVO_POSITION_OUT  = 600; // mm // TODO
    float bottomPusherPosition = BOTTOM_PUSHER_SERVO_POSITION_READY;

    // TRAPDOOR SERVO (NORMAL POSITION SERVO)
    static constexpr int TRAPDOOR_SERVO_ID = 4; // TODO
    static constexpr int TRAPDOOR_SERVO_OPEN   = 0;     // ° [0,270] // TODO
    static constexpr int TRAPDOOR_SERVO_CLOSED = 90;    // ° [0,270] // TODO

    static constexpr bool IS_COLOR_SENSOR_UNDERNEATH_BOXES = true; // in this case we must inverse the color seen
    com_types::TeamColor teamColor = com_types::TeamColor::UNKNOWN;
    ColorSensorTCS34725 colorSensor;

    void _closeTrapdoor();
    void _openTrapdoor();

    void _movePusherToPosition(int servoID, int speed, float target, float currentPosition);
};

#endif //STM_MAIN_BOARD_BOXESSORTER_H