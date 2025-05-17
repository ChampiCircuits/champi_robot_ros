#ifndef INC_SCSERVOS_H_
#define INC_SCSERVOS_H_

#include "stm32g4xx_hal.h"
#include "Devices/SCServos.h"

#define N_SERVOS 1

#define ID_SERVO_TEST 7


class SCServos {
public:
    SCServos() = default;
    explicit SCServos(UART_HandleTypeDef *huart);
    int EnableTorque(uint8_t ID, uint8_t Enable, uint8_t ReturnLevel=1);
    int WritePos(uint8_t ID, int position, int velocity, uint8_t ReturnLevel=1);
    int RegWritePos(uint8_t ID, int position, int velocity, uint8_t ReturnLevel=1);
    int ReadPos(uint8_t ID);
    int ReadVoltage(uint8_t ID);
    int ReadTemper(uint8_t ID);
    void RegWriteAction();
    void SyncWritePos(uint8_t ID[], uint8_t IDN, int position, int velocity);
    int WriteID(uint8_t oldID, uint8_t newID, uint8_t ReturnLevel=1);
    int WriteLimitAngle(uint8_t ID, int MinAngel, int MaxAngle, uint8_t ReturnLevel=1);
    int WriteLimitTroque(uint8_t ID, int MaxTroque, uint8_t ReturnLevel=1);
    int WritePunch(uint8_t ID, int Punch, uint8_t ReturnLevel=1);
    int WriteBaund(uint8_t ID, uint8_t Baund, uint8_t ReturnLevel=1);
    int WriteDeadBand(uint8_t ID, uint8_t CWDB, uint8_t CCWDB, uint8_t ReturnLevel=1);
    int WritePID(uint8_t ID, uint8_t P, uint8_t I, uint8_t D, uint8_t ReturnLevel=1);
    int WriteSpe(uint8_t ID, int velocity, uint8_t ReturnLevel=1);
    int LockEprom(uint8_t ID, uint8_t Enable, uint8_t ReturnLevel=1);
    int WriteIMax(uint8_t ID, int IMax, uint8_t ReturnLevel=1);
    void RotateClockwise();
    void RotateCounterClockwise();
    void scan_ids(uint8_t id_start,  uint8_t id_stop);
private:
    int	ReadBuf(uint16_t len, uint8_t *buf=nullptr) const;
    void fflushRevBuf() const;
    void write_byte(uint8_t reg) const;
    void write_bytes(uint8_t *reg, uint16_t len) const;
    UART_HandleTypeDef *huart_;

    uint8_t buffer[30]{};


#define		startByte	0xFF
#define		TIMEOUT		500//TIMEOUT 500ms

#define		B_1M		0
#define		B_0_5M		1
#define		B_250K		2
#define		B_128K		3
#define		B_115200	4
#define		B_76800		5
#define		B_57600		6
#define		B_38400		7
#define 	RX_PIN		10
#define 	TX_PIN		11
#define 	HARD_MODE	1
#define 	SOFT_MODE	2

    //register Address
#define P_MODEL_NUMBER_L 0
#define P_MODEL_NUMBER_H 1
#define P_VERSION_L 3
#define P_VERSION_H 4
#define P_ID 5
#define P_BAUD_RATE 6
#define P_RETURN_DELAY_TIME 7
#define P_RETURN_LEVEL 8
#define P_MIN_ANGLE_LIMIT_L 9
#define P_MIN_ANGLE_LIMIT_H 10
#define P_MAX_ANGLE_LIMIT_L 11
#define P_MAX_ANGLE_LIMIT_H 12
#define P_LIMIT_TEMPERATURE 13
#define P_MAX_LIMIT_VOLTAGE 14
#define P_MIN_LIMIT_VOLTAGE 15
#define P_MAX_TORQUE_L 16
#define P_MAX_TORQUE_H 17
#define P_ALARM_LED 18
#define P_ALARM_SHUTDOWN 19
#define P_COMPLIANCE_P 21
#define P_COMPLIANCE_D 22
#define P_COMPLIANCE_I 23
#define P_PUNCH_L 24
#define P_PUNCH_H 25
#define P_CW_DEAD 26
#define P_CCW_DEAD 27
#define P_IMAX_L 28
#define P_IMAX_H 29
#define P_OFFSET_L 30
#define P_OFFSET_H 31

#define P_TORQUE_ENABLE 40
#define P_LED 41
#define P_GOAL_POSITION_L 42
#define P_GOAL_POSITION_H 43
#define P_GOAL_SPEED_L 44
#define P_GOAL_SPEED_H 45
#define P_LOCK 48

#define P_PRESENT_POSITION_L 56
#define P_PRESENT_POSITION_H 57
#define P_PRESENT_SPEED_L 58
#define P_PRESENT_SPEED_H 59
#define P_PRESENT_LOAD_L 60
#define P_PRESENT_LOAD_H 61
#define P_PRESENT_VOLTAGE 62
#define P_PRESENT_TEMPERATURE 63
#define P_REGISTERED_INSTRUCTION 64
#define P_ERROR 65
#define P_MOVING 66
#define P_VIR_POSITION_L 67
#define P_VIR_POSITION_H 68
#define P_CURRENT_L 69
#define P_CURRENT_H 70

    //Instruction:
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_ACTION 0x05
#define INST_RESET 0x06
#define INST_SYNC_WRITE 0x83
};



namespace devices
{
    namespace scs_servos {

        extern uint8_t ids_servos[N_SERVOS];
        extern SCServos servos;
        extern bool init_successful;

        int test();
        void set_enable(bool enable);
        float read_angle(uint8_t id);
        void set_angle(uint8_t id, float angle, int ms);
        void set_angle_async(uint8_t id, float angle, int ms);
        void test_angle(uint8_t id, float angle); // Tests a given angle, then reverts it to original angle.
    }
}



#endif /* INC_SCSERVOS_H_ */
