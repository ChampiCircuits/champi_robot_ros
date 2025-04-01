#include "Devices/QwiicOTOS.h"
#include <cmath>
#include <cstring>

bool QwiicOTOS::isConnected() {
  auto a = HAL_I2C_IsDeviceReady(hi2c_, address << 1, 3, 1000);
  if (a != HAL_OK) {
    return false;
  }
  return true; // TODO
  return readByte(kRegProductId) == kProductId;
}

bool QwiicOTOS::begin() { return isConnected(); }

bool QwiicOTOS::selfTest() {
  if (!writeByte(kRegSelfTest, 0x01)) {
    return false;
  }

  HAL_Delay(5);
  uint8_t regValue;
  for (int i = 0; i < 10; ++i) {
    regValue = readByte(kRegSelfTest);
    if (((regValue >> 1) & 0x01) == 0) {
      break;
    }
    HAL_Delay(5);
  }
  return ((regValue >> 2) & 0x01) == 1;
}

bool QwiicOTOS::calibrateImu(uint8_t numSamples, bool waitUntilDone) {
  if (!writeByte(kRegImuCalib, numSamples)) {
    return false;
  }

  HAL_Delay(3);

  if (!waitUntilDone) {
    return true;
  }

  for (uint8_t i = numSamples; i > 0; --i) {
    if (readByte(kRegImuCalib) == 0) {
      return true;
    }
    HAL_Delay(3);
  }
  return false;
}

uint8_t QwiicOTOS::getImuCalibrationProgress() {
  return readByte(kRegImuCalib);
}

uint16_t QwiicOTOS::getVersionInfo() {
  uint8_t versionData[2] = {0};
  readBlock(kRegHwVersion, versionData, 2);
  return (versionData[0] << 8) | versionData[1];
}

QwiicOTOS::QwiicOTOS(I2C_HandleTypeDef *hi2c_, uint8_t addr)
    : hi2c_(hi2c_), address(addr) {}

float QwiicOTOS::getLinearScalar() {
  int8_t rawScalar = readByte(kRegScalarLinear);
  return rawScalar * 0.001f + 1.0f;
}

bool QwiicOTOS::setLinearScalar(float scalar) {
  if (scalar < kMinScalar || scalar > kMaxScalar)
    return false;
  int8_t rawScalar = static_cast<int8_t>((scalar - 1.0f) * 1000);
  writeByte(kRegScalarLinear, rawScalar);
  return true;
}

float QwiicOTOS::getAngularScalar() {
  int8_t rawScalar = readByte(kRegScalarAngular);
  return rawScalar * 0.001f + 1.0f;
}

bool QwiicOTOS::setAngularScalar(float scalar) {
  if (scalar < kMinScalar || scalar > kMaxScalar)
    return false;
  int8_t rawScalar = static_cast<int8_t>((scalar - 1.0f) * 1000) + 0.5f;
  return writeByte(kRegScalarAngular, rawScalar);
}

void QwiicOTOS::resetTracking() { writeByte(kRegReset, 0x01); }

// Lecture de la configuration du traitement du signal
int QwiicOTOS::getSignalProcessConfig() {
  return readByte(kRegSignalProcess); // Utilisation de la méthode i2cReadByte
                                      // pour lire un octet du registre
}

// Écriture de la configuration du traitement du signal
void QwiicOTOS::setSignalProcessConfig(int config) {
  writeByte(kRegSignalProcess,
            static_cast<uint8_t>(config)); // Conversion du config en uint8_t et
                                           // écriture dans le registre
}

// Lecture des offsets de position (en mètres et radians)
Pose2D QwiicOTOS::getOffset() {
  return readPoseRegs(
      kRegOffXL, kInt16ToMeter,
      kInt16ToRad); // Appel à la méthode de lecture des registres
}

// Écriture des offsets de position
bool QwiicOTOS::setOffset(const Pose2D &pose) {
  return writePoseRegs(
      kRegOffXL, pose, kMeterToInt16,
      kRadToInt16); // Conversion et écriture des données d'offset
}

// Lecture de la position (en mètres et radians)
Pose2D QwiicOTOS::getPosition() {
  return readPoseRegs(
      kRegPosXL, kInt16ToMeter,
      kInt16ToRad); // Lecture de la position depuis les registres
}

// Écriture de la position (en mètres et radians)
void QwiicOTOS::setPosition(const Pose2D &pose) {
  writePoseRegs(kRegPosXL, pose, kMeterToInt16,
                kRadToInt16); // Conversion et écriture de la nouvelle position
}

Pose2D QwiicOTOS::getPositionStdDev() {
  return readPoseRegs(kRegPosStdXL, kInt16ToMeter, kInt16ToRad);
}

bool QwiicOTOS::writeByte(uint8_t reg, uint8_t data) {
  return HAL_I2C_Mem_Write(hi2c_, address << 1, reg, I2C_MEMADD_SIZE_8BIT,
                           &data, 1, HAL_MAX_DELAY) == HAL_OK;
}

uint8_t QwiicOTOS::readByte(uint8_t reg) {
  uint8_t data = 0;
  HAL_I2C_Mem_Read(hi2c_, address << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1,
                   HAL_MAX_DELAY);
  return data;
}

bool QwiicOTOS::readBlock(uint8_t reg, uint8_t *data, uint16_t length) {
  return HAL_I2C_Mem_Read(hi2c_, address << 1, reg, I2C_MEMADD_SIZE_8BIT, data,
                          length, HAL_MAX_DELAY) == HAL_OK;
}

bool QwiicOTOS::writeBlock(uint8_t reg, uint8_t *data, uint16_t length) {
  return HAL_I2C_Mem_Write(hi2c_, address << 1, reg, I2C_MEMADD_SIZE_8BIT, data,
                           length, HAL_MAX_DELAY) == HAL_OK;
}

Pose2D QwiicOTOS::readPoseRegs(uint8_t reg, float rawToXY, float rawToH) {
  uint8_t rawData[6];
  readBlock(reg, rawData, 6);
  return regsToPose(rawData, rawToXY, rawToH);
}

Pose2D QwiicOTOS::regsToPose(const uint8_t *rawData, float rawToXY,
                             float rawToH) {
  int16_t rawX = (rawData[1] << 8) | rawData[0];
  int16_t rawY = (rawData[3] << 8) | rawData[2];
  int16_t rawH = (rawData[5] << 8) | rawData[4];
  return Pose2D(rawX * rawToXY * meterToUnit, rawY * rawToXY * meterToUnit,
                rawH * rawToH * radToUnit);
}

void QwiicOTOS::poseToRegs(const Pose2D &pose, uint8_t *rawData, float xyToRaw,
                           float hToRaw) {
  // Convert pose units to raw data
  int16_t rawX = pose.x * xyToRaw / meterToUnit;
  int16_t rawY = pose.y * xyToRaw / meterToUnit;
  int16_t rawH = pose.h * hToRaw / radToUnit;

  // Store raw data in buffer
  rawData[0] = rawX & 0xFF;
  rawData[1] = (rawX >> 8) & 0xFF;
  rawData[2] = rawY & 0xFF;
  rawData[3] = (rawY >> 8) & 0xFF;
  rawData[4] = rawH & 0xFF;
  rawData[5] = (rawH >> 8) & 0xFF;
}

int QwiicOTOS::writePoseRegs(uint8_t reg, const Pose2D &pose, float xyToRaw,
                             float hToRaw) {
  // Store raw data in a temporary buffer
  uint8_t rawData[6];
  poseToRegs(pose, rawData, xyToRaw, hToRaw);
  // Write the raw data to the device
  return writeBlock(reg, rawData, 6);
}


