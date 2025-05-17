#include <Devices/SCServos.h>
#include <cstdio>
#include "Util/logging.h"

#define UNIT_TO_DEG 0.26
#define DEG_TO_UNIT 3.79

namespace devices
{
    namespace scs_servos {

        uint8_t ids_servos[N_SERVOS] = {ID_SERVO_TEST};
        SCServos servos;
        bool init_successful = false;

        void find_ids(uint8_t from_id, uint8_t to_id) {
            for (uint8_t id=from_id; id<=to_id; id++) {
                if (servos.ReadPos(id) != -1) {
                	HAL_Delay(100);
                    LOG_INFO("scs", "Found servo: %d, pos = %d", id, servos.ReadPos(id));
                    HAL_Delay(100);
                }
            }
        }

        int test() {
            int result = 0;
            for (int i = 0; i < N_SERVOS; i++) {
                if (servos.ReadPos(ids_servos[i]) == -1) {
                    LOG_ERROR("scs", "Error reading servo number %d", ids_servos[i]);
                    result = -1;
                } else {
                    LOG_INFO("scs", "Servo number %d read successful", ids_servos[i]);
                }
                HAL_Delay(1);
            }
            return result;
        }

        void set_enable(bool enable) {
            for (const auto id : ids_servos) {
                servos.EnableTorque(id, enable);
                HAL_Delay(1);
            }
        }

        float read_angle(uint8_t id)
        {
            return static_cast<float>(servos.ReadPos(id)) * UNIT_TO_DEG;
        }

        void set_angle(uint8_t id, float angle, int ms) {
            set_angle_async(id, angle, ms);
            HAL_Delay(ms);
        }

        void set_angle_async(uint8_t id, float angle, int ms) {
            int position = (int)(angle * DEG_TO_UNIT);
            if (position < 0) {
                position = 0;
            } else if (position > 1023) {
                position = 1023;
            }
            servos.WritePos(id, position, ms);
            HAL_Delay(1);
        }

        void test_angle(uint8_t id, float angle) {
            int pos = servos.ReadPos(id);
            if (pos == -1) {
                LOG_ERROR("scs", "Error reading servo number %d", id);
                return;
            }
            set_angle(id, angle, 1000);
            HAL_Delay(1500);
            set_angle(id, pos * UNIT_TO_DEG, 1000);
            HAL_Delay(1500);

        }


    }
}


SCServos::SCServos (UART_HandleTypeDef *huart) : huart_(huart) {

}

void SCServos::write_byte(uint8_t reg) const {
    HAL_UART_Transmit(huart_, &reg, 1, 10);
    uint8_t data;
    HAL_UART_Receive(huart_, &data, 1, 10); // Cause we receive sent bytes (single wire)
}

void SCServos::write_bytes(uint8_t *reg, uint16_t len) const {
    for (uint16_t i = 0; i < len; i++) {
        write_byte(reg[i]);
    }
}

int SCServos::ReadBuf(uint16_t len, uint8_t *buf) const {
    int ret = HAL_UART_Receive(huart_, buf, len, 1000);
    if(ret==HAL_OK) {
        return len;
    }
    return -1;
}

void SCServos::fflushRevBuf() const {
	uint8_t data;
	while(HAL_UART_Receive(huart_, &data, 1, 0)==HAL_OK);
}

int SCServos::EnableTorque(uint8_t ID, uint8_t Enable, uint8_t ReturnLevel)
{
    int messageLength = 4;

    fflushRevBuf();

    buffer[0] = startByte;
    buffer[1] = startByte;
    buffer[2] = ID;
    buffer[3] = messageLength;
    buffer[4] = INST_WRITE;
    buffer[5] = P_TORQUE_ENABLE;
    buffer[6] = Enable;
    buffer[7] = (~(ID + messageLength + INST_WRITE + Enable + P_TORQUE_ENABLE))&0xFF;
    write_bytes(buffer, 8);
    if(ID !=  0xfe && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServos::WritePos(uint8_t ID, int position, int velocity, uint8_t ReturnLevel)
{
    int messageLength = 7;
    uint8_t posL = position>>8;
    uint8_t posH = position&0xff;
    uint8_t velL = velocity>>8;
    uint8_t velH = velocity&0xff;

    fflushRevBuf();
    buffer[0] = startByte;
    buffer[1] = startByte;
    buffer[2] = ID;
    buffer[3] = messageLength;
    buffer[4] = INST_WRITE;
    buffer[5] = P_GOAL_POSITION_L;
    buffer[6] = posL;
    buffer[7] = posH;
    buffer[8] = velL;
    buffer[9] = velH;
    buffer[10] = (~(ID + messageLength + INST_WRITE + P_GOAL_POSITION_L + posL + posH + velL + velH)) & 0xFF;
    write_bytes(buffer, 11);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServos::RegWritePos(uint8_t ID, int position, int velocity, uint8_t ReturnLevel)
{
    int messageLength = 7;
    uint8_t posL = position>>8;
    uint8_t posH = position&0xff;
    uint8_t velL = velocity>>8;
    uint8_t velH = velocity&0xff;

    fflushRevBuf();
    write_byte(startByte);
    write_byte(startByte);
    write_byte(ID);
    write_byte(messageLength);
    write_byte(INST_REG_WRITE);
    write_byte(P_GOAL_POSITION_L);
    write_byte(posL);
    write_byte(posH);
    write_byte(velL);
    write_byte(velH);
    write_byte((~(ID + messageLength + INST_REG_WRITE + P_GOAL_POSITION_L + posL + posH + velL + velH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

void SCServos::RegWriteAction()
{
    int messageLength = 2;
    uint8_t ID = 16;
    buffer[0] = startByte;
    buffer[1] = startByte;
    buffer[2] = ID;
    buffer[3] = messageLength;
    buffer[4] = INST_ACTION;
    buffer[5] = (~(ID + messageLength + INST_ACTION)) & 0xFF;
    write_bytes(buffer, 6);
}

int SCServos::ReadPos(uint8_t ID)
{
    uint8_t buf[8] = {0};
    int size;
    int pos=0;

    fflushRevBuf();
    buffer[0] = startByte;
    buffer[1] = startByte;
    buffer[2] = ID;
    buffer[3] = 4;
    buffer[4] = INST_READ;
    buffer[5] = P_PRESENT_POSITION_L;
    buffer[6] = 2;
    buffer[7] = (~(ID + 4 + INST_READ + P_PRESENT_POSITION_L + 2)) & 0xFF;
    write_bytes(buffer, 8);
    size = ReadBuf(8, buf);
    if(size<8)
        return -1;
    pos = buf[5];
    pos <<= 8;
    pos |= buf[6];
    return pos;
}

void SCServos::SyncWritePos(uint8_t ID[], uint8_t IDN, int position, int velocity)
{
    int messageLength = 5*IDN+4;
    uint8_t Sum = 0;
    uint8_t posL = position>>8;
    uint8_t posH = position&0xff;

    uint8_t velL = velocity>>8;
    uint8_t velH = velocity&0xff;

    write_byte(startByte);
    write_byte(startByte);
    write_byte(16);
    write_byte(messageLength);
    write_byte(INST_SYNC_WRITE);
    write_byte(P_GOAL_POSITION_L);
    write_byte(4);

    Sum = 16 + messageLength + INST_SYNC_WRITE + P_GOAL_POSITION_L + 4;
    int i;
    for(i=0; i<IDN; i++){
        write_byte(ID[i]);
        write_byte(posL);
        write_byte(posH);
        write_byte(velL);
        write_byte(velH);
        Sum += ID[i] + posL + posH + velL + velH;
    }
    write_byte((~Sum)&0xFF);
}

int SCServos::WriteID(uint8_t oldID, uint8_t newID, uint8_t ReturnLevel)
{
    int messageLength = 4;

    fflushRevBuf();
    write_byte(startByte);
    write_byte(startByte);
    write_byte(oldID);
    write_byte(messageLength);
    write_byte(INST_WRITE);
    write_byte(P_ID);
    write_byte(newID);
    write_byte((~(oldID + messageLength + INST_WRITE + newID + P_ID))&0xFF);
    if(oldID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServos::WriteLimitAngle(uint8_t ID, int MinAngel, int MaxAngle, uint8_t ReturnLevel)
{
    int messageLength = 7;
    uint8_t MinAL = MinAngel>>8;
    uint8_t MinAH = MinAngel&0xff;
    uint8_t MaxAL = MaxAngle>>8;
    uint8_t MaxAH = MaxAngle&0xff;

    fflushRevBuf();
    write_byte(startByte);
    write_byte(startByte);
    write_byte(ID);
    write_byte(messageLength);
    write_byte(INST_WRITE);
    write_byte(P_MIN_ANGLE_LIMIT_L);
    write_byte(MinAL);
    write_byte(MinAH);
    write_byte(MaxAL);
    write_byte(MaxAH);
    write_byte((~(ID + messageLength + INST_WRITE + P_MIN_ANGLE_LIMIT_L + MinAL + MinAH + MaxAL + MaxAH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServos::WriteLimitTroque(uint8_t ID, int MaxTroque, uint8_t ReturnLevel)
{
    int messageLength = 5;
    uint8_t MaxTL = MaxTroque>>8;
    uint8_t MaxTH = MaxTroque&0xff;

    fflushRevBuf();
    buffer[0] = startByte;
    buffer[1] = startByte;
    buffer[2] = ID;
    buffer[3] = messageLength;
    buffer[4] = INST_WRITE;
    buffer[5] = P_MAX_TORQUE_L;
    buffer[6] = MaxTL;
    buffer[7] = MaxTH;
    buffer[8] = (~(ID + messageLength + INST_WRITE + P_MAX_TORQUE_L + MaxTL + MaxTH)) & 0xFF;
    write_bytes(buffer, 9);

    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServos::WritePunch(uint8_t ID, int Punch, uint8_t ReturnLevel)
{
    int messageLength = 5;
    uint8_t PunchL = Punch>>8;
    uint8_t PunchH = Punch&0xff;

    fflushRevBuf();
    write_byte(startByte);
    write_byte(startByte);
    write_byte(ID);
    write_byte(messageLength);
    write_byte(INST_WRITE);
    write_byte(P_PUNCH_L);
    write_byte(PunchL);
    write_byte(PunchH);

    write_byte((~(ID + messageLength + INST_WRITE + P_PUNCH_L + PunchL + PunchH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServos::WriteBaund(uint8_t ID, uint8_t Baund, uint8_t ReturnLevel)
{
    int messageLength = 4;

    fflushRevBuf();
    write_byte(startByte);
    write_byte(startByte);
    write_byte(ID);
    write_byte(messageLength);
    write_byte(INST_WRITE);
    write_byte(P_BAUD_RATE);
    write_byte(Baund);

    write_byte((~(ID + messageLength + INST_WRITE + P_BAUD_RATE + Baund))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServos::WriteDeadBand(uint8_t ID, uint8_t CWDB, uint8_t CCWDB, uint8_t ReturnLevel)
{
    int messageLength = 5;

    fflushRevBuf();
    write_byte(startByte);
    write_byte(startByte);
    write_byte(ID);
    write_byte(messageLength);
    write_byte(INST_WRITE);
    write_byte(P_CW_DEAD);
    write_byte(CWDB);
    write_byte(CCWDB);

    write_byte((~(ID + messageLength + INST_WRITE + P_CW_DEAD + CWDB + CCWDB))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServos::WriteIMax(uint8_t ID, int IMax, uint8_t ReturnLevel)
{
    int messageLength = 5;

    uint8_t velL = IMax>>8;
    uint8_t velH = IMax&0xff;

    fflushRevBuf();
    write_byte(startByte);
    write_byte(startByte);
    write_byte(ID);
    write_byte(messageLength);
    write_byte(INST_WRITE);
    write_byte(P_IMAX_L);
    write_byte(velL);
    write_byte(velH);
    write_byte((~(ID + messageLength + INST_WRITE + P_IMAX_L + velL + velH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}


int SCServos::LockEprom(uint8_t ID, uint8_t Enable, uint8_t ReturnLevel)
{
    int messageLength = 4;

    fflushRevBuf();
    write_byte(startByte);
    write_byte(startByte);
    write_byte(ID);
    write_byte(messageLength);
    write_byte(INST_WRITE);
    write_byte(P_LOCK);
    write_byte(Enable);

    write_byte((~(ID + messageLength + INST_WRITE + P_LOCK + Enable))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServos::WritePID(uint8_t ID, uint8_t P, uint8_t I, uint8_t D, uint8_t ReturnLevel)
{
    int messageLength = 6;

    fflushRevBuf();
    write_byte(startByte);
    write_byte(startByte);
    write_byte(ID);
    write_byte(messageLength);
    write_byte(INST_WRITE);
    write_byte(P_COMPLIANCE_P);
    write_byte(P);
    write_byte(D);
    write_byte(I);

    write_byte((~(ID + messageLength + INST_WRITE + P_COMPLIANCE_P + P + D + I))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServos::WriteSpe(uint8_t ID, int velocity, uint8_t ReturnLevel)
{
    int messageLength = 5;

    int vel = velocity;
    if(velocity<0){
        vel = -velocity;
        vel |= (1<<10);
    }

    uint8_t velL =  vel>>8;
    uint8_t velH =  vel&0xff;

    fflushRevBuf();
    write_byte(startByte);
    write_byte(startByte);
    write_byte(ID);
    write_byte(messageLength);
    write_byte(INST_WRITE);
    write_byte(P_GOAL_SPEED_L);
    write_byte(velL);
    write_byte(velH);
    write_byte((~(ID + messageLength + INST_WRITE + P_GOAL_SPEED_L + velL + velH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}


int SCServos::ReadVoltage(uint8_t ID)
{
    uint8_t buf[7] = {0};
    uint8_t size;
    int vol;

    fflushRevBuf();
    write_byte(startByte);
    write_byte(startByte);
    write_byte(ID);
    write_byte(4);
    write_byte(INST_READ);
    write_byte(P_PRESENT_VOLTAGE);
    write_byte(1);
    write_byte((~(ID + 4 + INST_READ + P_PRESENT_VOLTAGE + 1))&0xFF);
    size = ReadBuf(7, buf);
    if(size<7)
        return -1;
    vol = buf[5];
    return vol;
}

int SCServos::ReadTemper(uint8_t ID)
{
    uint8_t buf[7] = {0};
    uint8_t size;
    int temper;

    fflushRevBuf();
    write_byte(startByte);
    write_byte(startByte);
    write_byte(ID);
    write_byte(4);
    write_byte(INST_READ);
    write_byte(P_PRESENT_TEMPERATURE);
    write_byte(1);
    write_byte((~(ID + 4 + INST_READ + P_PRESENT_TEMPERATURE + 1))&0xFF);
    size = ReadBuf(7, buf);
    if(size<7)
        return -1;
    temper = buf[5];
    return temper;
}

void SCServos::RotateClockwise(){
    EnableTorque(16, 1);
    HAL_Delay(200);
    WritePos(16, 400, 500);
}

void SCServos::RotateCounterClockwise(){
    EnableTorque(16, 1);
    HAL_Delay(200);
    WritePos(16, 750, 500);
}


void SCServos::scan_ids(uint8_t id_start,  uint8_t id_stop) {
    for(uint8_t id=id_start; id<id_stop; id++) {
        if(ReadPos(id)!=-1) {
            printf("Found ID %d\n", id);
        }
    }
}
