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
 * - 1 servo to release EXIT RAMP
 */


class BoxesSorter
{
public:
    void prepareTopPusher();
    void grabAndSort2BoxesFromLift();
    void push2BoxesOut();
    void openExitRamp();
    void initialize();
    void setTeamColor(com_types::TeamColor color);
    [[nodiscard]] com_types::TeamColor getTeamColor() const { return teamColor; }
    [[nodiscard]] bool isPusherReady() const { return topPusherPosition == TOP_PUSHER_SERVO_POSITION_READY; }
    void moveTopPusherToPosition(float target);
    void moveBottomPusherToPosition(float target);

    int boxesInSorterCount = 0; // always an even number (clamp takes boxes 2 by 2)

    static constexpr int TOP_PUSHER_SERVO_ID = 2; // TODO
    static constexpr int BOTTOM_PUSHER_SERVO_ID = 12; // ID ok
    static constexpr int TRAPDOOR_SERVO_ID = 4; // TODO
    static constexpr int EXIT_RAMP_SERVO_ID = 5; // TODO

private:
    static constexpr float BOX_WIDTH = 100; // encoder steps --> pusher increment // TODO calibrate!
    static constexpr int ENCODER_RANGE = 1024; // encoder wraps at 1024 (0-1023)

    // TOP PUSHER SERVO (FREE ROTATION SERVO)
    // | TOBOGGAN - STRAIGHT HOLE - COLOR SENSOR - LIFT - READY POSITION |
    static constexpr float TOP_PUSHER_SERVO_SPEED  = 1023;  // [0,1023]
    static constexpr float TOP_PUSHER_SERVO_POSITION_READY  = 0;
    static constexpr float TOP_PUSHER_SERVO_POSITION_COLOR_SENSOR_FIRST_BOX  = 400;  // steps // TODO
    static constexpr float TOP_PUSHER_SERVO_POSITION_STRAIGHT_HOLE_FIRST_BOX  = 600; // steps // TODO
    static constexpr float TOP_PUSHER_SERVO_POSITION_TOBOGGAN_HOLE_FIRST_BOX  = 800; // steps // TODO
    float topPusherPosition = TOP_PUSHER_SERVO_POSITION_READY;
    GPIO_TypeDef* TOP_END_SWITCH_GPIO_Port = D0_GPIO_Port;
    static constexpr auto TOP_END_SWITCH_GPIO_Pin = D0_Pin;

    // BOTTOM PUSHER SERVO (FREE ROTATION SERVO)
    // | OUT | TOBOGGAN - STRAIGHT HOLE - READY |
    static constexpr float BOTTOM_PUSHER_SERVO_SPEED  = 500;  // [0,1023]
    static constexpr float BOTTOM_PUSHER_SERVO_POSITION_READY  = -125;  // steps
    static constexpr float BOTTOM_PUSHER_SERVO_POSITION_OUT  = 1500; // steps // TODO calibrate!
    float bottomPusherPosition = BOTTOM_PUSHER_SERVO_POSITION_READY;
    GPIO_TypeDef* BOTTOM_END_SWITCH_GPIO_Port = D6_GPIO_Port;
    static constexpr auto BOTTOM_END_SWITCH_GPIO_Pin = D6_Pin;

    // TRAPDOOR SERVO (NORMAL POSITION SERVO)
    static constexpr int TRAPDOOR_SERVO_OPEN   = 0;     // ° [0,270] // TODO
    static constexpr int TRAPDOOR_SERVO_CLOSED = 90;    // ° [0,270] // TODO

    // EXIT RAMP SERVO (NORMAL POSITION SERVO)
    static constexpr int EXIT_RAMP_SERVO_IDLE   = 0;     // ° [0,270] // TODO
    static constexpr int EXIT_RAMP_SERVO_RELEASED = 90;    // ° [0,270] // TODO

    // COLOR SENSOR
    static constexpr bool IS_COLOR_SENSOR_UNDERNEATH_BOXES = true; // in this case we must inverse the color seen
    com_types::TeamColor teamColor = com_types::TeamColor::UNKNOWN;
    ColorSensorTCS34725 colorSensor;

    void _closeTrapdoor();
    void _openTrapdoor();

    void _movePusherToPosition(int servoID, int speed, float target, float currentPosition);
};

#endif //STM_MAIN_BOARD_BOXESSORTER_H