#ifndef STM_MAIN_BOARD_LIFTANDCLAMP_H
#define STM_MAIN_BOARD_LIFTANDCLAMP_H

#include "main.h"
#include "Application/PosSteppersTask.h"


class LiftAndClamp
{
public:
    void take2Boxes();
    void put2LastBoxesOnTheGround();
    void initialization();

    int boxesCount = 0; // always an even number (clamp takes boxes 2 by 2)

private:
    // LIFT
    static constexpr float LIFT_INIT_ACCEL = 5.0;
    static constexpr float LIFT_INIT_SPEED = 5.0;
    static constexpr float LIFT_BASE_ACCEL = 20.0;
    static constexpr float LIFT_BASE_SPEED = 15.0;

    static constexpr float LIFT_BOTTOM_POSITION = 0.0;
    static constexpr float LIFT_TOP_POSITION    = 150.0; // TODO
    static constexpr float BOX_HEIGHT = 150.0; // --> lift height increment // TODO

    // END SWITCH
    static const inline GPIO_TypeDef* END_SWITCH_GPIO_Port = D6_GPIO_Port; // TODO
    static constexpr auto END_SWITCH_GPIO_Pin = D6_Pin;      // TODO

    // CLAMP
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
};


#endif //STM_MAIN_BOARD_LIFTANDCLAMP_H