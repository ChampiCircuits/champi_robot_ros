#ifndef STM_MAIN_BOARD_LIFTANDCLAMP_H
#define STM_MAIN_BOARD_LIFTANDCLAMP_H

#include "main.h"
#include "Application/PosSteppersTask.h"

/**
 * Lift and CLAMP actuators :
 * - LIFT = 1 stepper + 1 limit switch to init position
 * - CLAMP = 1 servo to clamp the NutBox
 */


class LiftAndClamp
{
public:
    void take2Boxes();
    void bring2BoxesToTop();
    void put2LastBoxesOnTheGround();
    void initialize();

    /** Returns true after bring2BoxesToTop() and until markBoxesGrabbed() is called. */
    [[nodiscard]] bool hasBoxesReadyAtTop() const { return _boxesReadyAtTop; }
    /** Call this after grabAndSort2BoxesFromLift() to signal the boxes have been taken. */
    void markBoxesGrabbed() { _boxesReadyAtTop = false; }

    int boxesInLiftCount = 0; // always an even number (clamp takes boxes 2 by 2)

private:
    // LIFT STEPPER
    static constexpr float LIFT_INIT_ACCEL = 5.0;
    static constexpr float LIFT_INIT_SPEED = 5.0;
    static constexpr float LIFT_BASE_ACCEL = 20.0;
    static constexpr float LIFT_BASE_SPEED = 15.0;

    static constexpr float LIFT_BOTTOM_POSITION = 0.0;
    static constexpr float LIFT_TOP_POSITION    = 150.0; // TODO
    static constexpr float BOX_HEIGHT = 30.0; // mm --> lift height increment

    // END SWITCH
    GPIO_TypeDef* END_SWITCH_GPIO_Port = D6_GPIO_Port; // TODO
    static constexpr auto END_SWITCH_GPIO_Pin = D6_Pin;      // TODO

    // CLAMP SERVO
    static constexpr int CLAMP_SERVO_ID = 0;                     // TODO
    static constexpr int CLAMP_SERVO_OPEN   = 0;    // ° [0,270] // TODO
    static constexpr int CLAMP_SERVO_CLOSED = 150;  // ° [0,270] // TODO

    void _initLift();
    void _homingLift();
    void _initClamp();

    void _closeClamp(bool async = false);
    void _releaseClamp(bool async = false);

    void _liftGoToPosition(float position, bool async = false);
    void _liftGoToBottom(bool async = false) { _liftGoToPosition(LIFT_BOTTOM_POSITION, async); }

    bool _boxesReadyAtTop = false;
};


#endif //STM_MAIN_BOARD_LIFTANDCLAMP_H
