#pragma once
namespace can_ids {
    int BASE_CMD_VEL = 0x10;
    int BASE_CURRENT_VEL = 0x11;
    int BASE_STATUS = 0x12;
    int BASE_SET_CONFIG = 0x13;
    int BASE_RET_CONFIG = 0x14;
    int BASE_GET_LOG = 0x15;
    int BASE_RET_LOG = 0x16;
    int BASE_RESET = 0x1;
    int BASE_TEST = 0x200;
    int RET_BASE_TEST = 0x201;
    int IMU_DATA = 0x20;
    int IMU_STATUS = 0x21;
    int IMU_TEST = 0x202;
    int ACT_STATUS =  0x31;
    int ACT_TEST =  0x203;
    int ACT_ACTION =  0x32;
    int EMERGENCY_STOP = 0x5;
    int TIRETTE_START = 0x6;
    int ACT_RESET = 0x2;
    int LED_RING_DISTANCES =  0x210;
    int TRACKING_SENSOR_DATA =  0x70;
    int TRACKING_SENSOR_STD =  0x71;
    int RESET_AND_CALIBRATE_TRACKING_SENSOR =  0x72;
}
