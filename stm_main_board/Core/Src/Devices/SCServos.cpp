#include "cmsis_os2.h"

#include <Devices/SCServos.h>
#include <cstdio>

SCServos::SCServos (UART_HandleTypeDef *huart) : huart_(huart)
{
}

void SCServos::Printf(uint8_t reg)
{
    HAL_UART_Transmit(huart_, &reg, 1, 10);
    uint8_t data;
    HAL_UART_Receive(huart_, &data, 1, 10); // Cause we receive sent bytes (single wire)
}

void SCServos::fflushRevBuf()
{
	uint8_t data;
	while(HAL_UART_Receive(huart_, &data, 1, 0)==HAL_OK);
}

int SCServos::EnableTorque(uint8_t ID, uint8_t Enable, uint8_t ReturnLevel)
{
    int messageLength = 4;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_TORQUE_ENABLE);
    Printf(Enable);
    Printf((~(ID + messageLength + INST_WRITE + Enable + P_TORQUE_ENABLE))&0xFF);
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
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_GOAL_POSITION_L);
    Printf(posL);
    Printf(posH);
    Printf(velL);
    Printf(velH);
    Printf((~(ID + messageLength + INST_WRITE + P_GOAL_POSITION_L + posL + posH + velL + velH))&0xFF);
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
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_REG_WRITE);
    Printf(P_GOAL_POSITION_L);
    Printf(posL);
    Printf(posH);
    Printf(velL);
    Printf(velH);
    Printf((~(ID + messageLength + INST_REG_WRITE + P_GOAL_POSITION_L + posL + posH + velL + velH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

void SCServos::RegWriteAction()
{
    int messageLength = 2;
    uint8_t ID = 16;
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_ACTION);
    Printf((~(ID + messageLength + INST_ACTION))&0xFF);
}

int SCServos::ReadBuf(uint16_t len, uint8_t *buf)
{
	int ret = HAL_UART_Receive(huart_, buf, len, 1000);
    if(ret==HAL_OK) {
    	return len;
    }
    return -1;

}

int SCServos::ReadPos(uint8_t ID)
{
    uint8_t buf[8] = {0};
    int size;
    int pos=0;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(4);
    Printf(INST_READ);
    Printf(P_PRESENT_POSITION_L);
    Printf(2);
    Printf((~(ID + 4 + INST_READ + P_PRESENT_POSITION_L + 2))&0xFF);
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

    Printf(startByte);
    Printf(startByte);
    Printf(16);
    Printf(messageLength);
    Printf(INST_SYNC_WRITE);
    Printf(P_GOAL_POSITION_L);
    Printf(4);

    Sum = 16 + messageLength + INST_SYNC_WRITE + P_GOAL_POSITION_L + 4;
    int i;
    for(i=0; i<IDN; i++){
        Printf(ID[i]);
        Printf(posL);
        Printf(posH);
        Printf(velL);
        Printf(velH);
        Sum += ID[i] + posL + posH + velL + velH;
    }
    Printf((~Sum)&0xFF);
}

int SCServos::WriteID(uint8_t oldID, uint8_t newID, uint8_t ReturnLevel)
{
    int messageLength = 4;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(oldID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_ID);
    Printf(newID);
    Printf((~(oldID + messageLength + INST_WRITE + newID + P_ID))&0xFF);
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
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_MIN_ANGLE_LIMIT_L);
    Printf(MinAL);
    Printf(MinAH);
    Printf(MaxAL);
    Printf(MaxAH);
    Printf((~(ID + messageLength + INST_WRITE + P_MIN_ANGLE_LIMIT_L + MinAL + MinAH + MaxAL + MaxAH))&0xFF);
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
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_MAX_TORQUE_L);
    Printf(MaxTL);
    Printf(MaxTH);

    Printf((~(ID + messageLength + INST_WRITE + P_MAX_TORQUE_L + MaxTL + MaxTH))&0xFF);
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
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_PUNCH_L);
    Printf(PunchL);
    Printf(PunchH);

    Printf((~(ID + messageLength + INST_WRITE + P_PUNCH_L + PunchL + PunchH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServos::WriteBaund(uint8_t ID, uint8_t Baund, uint8_t ReturnLevel)
{
    int messageLength = 4;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_BAUD_RATE);
    Printf(Baund);

    Printf((~(ID + messageLength + INST_WRITE + P_BAUD_RATE + Baund))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServos::WriteDeadBand(uint8_t ID, uint8_t CWDB, uint8_t CCWDB, uint8_t ReturnLevel)
{
    int messageLength = 5;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_CW_DEAD);
    Printf(CWDB);
    Printf(CCWDB);

    Printf((~(ID + messageLength + INST_WRITE + P_CW_DEAD + CWDB + CCWDB))&0xFF);
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
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_IMAX_L);
    Printf(velL);
    Printf(velH);
    Printf((~(ID + messageLength + INST_WRITE + P_IMAX_L + velL + velH))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}


int SCServos::LockEprom(uint8_t ID, uint8_t Enable, uint8_t ReturnLevel)
{
    int messageLength = 4;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_LOCK);
    Printf(Enable);

    Printf((~(ID + messageLength + INST_WRITE + P_LOCK + Enable))&0xFF);
    if(ID != 16 && ReturnLevel==2)
        return ReadBuf(6);
    return 0;
}

int SCServos::WritePID(uint8_t ID, uint8_t P, uint8_t I, uint8_t D, uint8_t ReturnLevel)
{
    int messageLength = 6;

    fflushRevBuf();
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_COMPLIANCE_P);
    Printf(P);
    Printf(D);
    Printf(I);

    Printf((~(ID + messageLength + INST_WRITE + P_COMPLIANCE_P + P + D + I))&0xFF);
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
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(messageLength);
    Printf(INST_WRITE);
    Printf(P_GOAL_SPEED_L);
    Printf(velL);
    Printf(velH);
    Printf((~(ID + messageLength + INST_WRITE + P_GOAL_SPEED_L + velL + velH))&0xFF);
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
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(4);
    Printf(INST_READ);
    Printf(P_PRESENT_VOLTAGE);
    Printf(1);
    Printf((~(ID + 4 + INST_READ + P_PRESENT_VOLTAGE + 1))&0xFF);
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
    Printf(startByte);
    Printf(startByte);
    Printf(ID);
    Printf(4);
    Printf(INST_READ);
    Printf(P_PRESENT_TEMPERATURE);
    Printf(1);
    Printf((~(ID + 4 + INST_READ + P_PRESENT_TEMPERATURE + 1))&0xFF);
    size = ReadBuf(7, buf);
    if(size<7)
        return -1;
    temper = buf[5];
    return temper;
}

void SCServos::RotateClockwise(){
    EnableTorque(16, 1);
    osDelay(200);
    WritePos(16, 400, 500);
}

void SCServos::RotateCounterClockwise(){
    EnableTorque(16, 1);
    osDelay(200);
    WritePos(16, 750, 500);
}


void SCServos::scan_ids(uint8_t id_start,  uint8_t id_stop) {
    for(uint8_t id=id_start; id<id_stop; id++) {
        if(ReadPos(id)!=-1) {
            printf("Found ID %d\n", id);
        }
    }
}
