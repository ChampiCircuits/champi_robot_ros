/*
 * QwiicOTOS.h
 *
 *  Created on: Mar 24, 2025
 *      Author: andre
 */

#ifndef INC_DEVICES_QWIICOTOS_H_
#define INC_DEVICES_QWIICOTOS_H_


#include "stm32h7xx_hal.h"
#include <tuple>

class Pose2D {
public:
    float x, y, h;

    Pose2D(float x = 0.0, float y = 0.0, float h = 0.0) : x(x), y(y), h(h) {}
};

/// @union sfe_otos_self_test_config_t
/// @brief Self test register bit fields
typedef union {
    struct
    {
        /// @brief Write 1 to start the self test
        uint8_t start : 1;

        /// @brief Returns 1 while the self test is in progress
        uint8_t inProgress : 1;

        /// @brief Returns 1 if the self test passed
        uint8_t pass : 1;

        /// @brief Returns 1 if the self test failed
        uint8_t fail : 1;

        /// @brief Reserved bits, do not use
        uint8_t reserved : 4;
    };

    /// @brief Raw register value
    uint8_t value;
} sfe_otos_self_test_config_t;

class QwiicOTOS {
public:

    // Adresses I2C
    static constexpr uint8_t kDefaultAddress = 0x17;

    // Constructeur
    QwiicOTOS(I2C_HandleTypeDef *hi2c, uint8_t address = kDefaultAddress);

    bool isConnected();
    bool begin();
    bool selfTest();
    bool calibrateImu(uint8_t numSamples = 255, bool waitUntilDone = true);
    uint8_t getImuCalibrationProgress();
    uint16_t getVersionInfo();

    int getAngularUnit() const;

    float getLinearScalar();
    bool setLinearScalar(float scalar);

    float getAngularScalar();
    bool setAngularScalar(float scalar);

    void resetTracking();

    int getSignalProcessConfig();
    void setSignalProcessConfig(int config);

    int getStatus();

    Pose2D getOffset();
    bool setOffset(const Pose2D& pose);

    Pose2D getPosition();
    void setPosition(const Pose2D& pose);

    Pose2D getVelocity();
    Pose2D getAcceleration();

    Pose2D getPositionStdDev();
    Pose2D getVelocityStdDev();
    Pose2D getAccelerationStdDev();

    std::tuple<Pose2D, Pose2D, Pose2D> getPosVelAcc();
    std::tuple<Pose2D, Pose2D, Pose2D> getPosVelAccStdDev();
    std::tuple<Pose2D, Pose2D, Pose2D, Pose2D, Pose2D, Pose2D> getPosVelAccAndStdDev();


private:
    I2C_HandleTypeDef *hi2c_;

    bool writeByte(uint8_t reg, uint8_t data);
    uint8_t readByte(uint8_t reg);
    bool readBlock(uint8_t reg, uint8_t *data, uint16_t length);
    bool writeBlock(uint8_t reg, uint8_t *data, uint16_t length);

    uint8_t address;
    float radToUnit = 1;
    float meterToUnit = 1;

    int writePoseRegs(uint8_t reg, const Pose2D& pose, float xyToRaw, float hToRaw);
    Pose2D readPoseRegs(uint8_t reg, float rawToXY, float rawToH);
    Pose2D regsToPose(const uint8_t* rawData, float rawToXY, float rawToH);
    void poseToRegs(const Pose2D& pose, uint8_t* rawData, float xyToRaw, float hToRaw);


    // Définition des constantes pour les registres
    const uint8_t kRegProductId = 0x00;
    const uint8_t kRegHwVersion = 0x01;
    const uint8_t kRegFwVersion = 0x02;
    const uint8_t kRegScalarLinear = 0x04;
    const uint8_t kRegScalarAngular = 0x05;
    const uint8_t kRegImuCalib = 0x06;
    const uint8_t kRegReset = 0x07;
    const uint8_t kRegSignalProcess = 0x0E;
    const uint8_t kRegSelfTest = 0x0F;
    const uint8_t kRegOffXL = 0x10;
    const uint8_t kRegOffXH = 0x11;
    const uint8_t kRegOffYL = 0x12;
    const uint8_t kRegOffYH = 0x13;
    const uint8_t kRegOffHL = 0x14;
    const uint8_t kRegOffHH = 0x15;
    const uint8_t kRegStatus = 0x1F;
    const uint8_t kRegPosXL = 0x20;
    const uint8_t kRegPosXH = 0x21;
    const uint8_t kRegPosYL = 0x22;
    const uint8_t kRegPosYH = 0x23;
    const uint8_t kRegPosHL = 0x24;
    const uint8_t kRegPosHH = 0x25;
    const uint8_t kRegVelXL = 0x26;
    const uint8_t kRegVelXH = 0x27;
    const uint8_t kRegVelYL = 0x28;
    const uint8_t kRegVelYH = 0x29;
    const uint8_t kRegVelHL = 0x2A;
    const uint8_t kRegVelHH = 0x2B;
    const uint8_t kRegAccXL = 0x2C;
    const uint8_t kRegAccXH = 0x2D;
    const uint8_t kRegAccYL = 0x2E;
    const uint8_t kRegAccYH = 0x2F;
    const uint8_t kRegAccHL = 0x30;
    const uint8_t kRegAccHH = 0x31;
    const uint8_t kRegPosStdXL = 0x32;
    const uint8_t kRegPosStdXH = 0x33;
    const uint8_t kRegPosStdYL = 0x34;
    const uint8_t kRegPosStdYH = 0x35;
    const uint8_t kRegPosStdHL = 0x36;
    const uint8_t kRegPosStdHH = 0x37;
    const uint8_t kRegVelStdXL = 0x38;
    const uint8_t kRegVelStdXH = 0x39;
    const uint8_t kRegVelStdYL = 0x3A;
    const uint8_t kRegVelStdYH = 0x3B;
    const uint8_t kRegVelStdHL = 0x3C;
    const uint8_t kRegVelStdHH = 0x3D;
    const uint8_t kRegAccStdXL = 0x3E;
    const uint8_t kRegAccStdXH = 0x3F;
    const uint8_t kRegAccStdYL = 0x40;
    const uint8_t kRegAccStdYH = 0x41;
    const uint8_t kRegAccStdHL = 0x42;
    const uint8_t kRegAccStdHH = 0x43;

    // Identifiant produit
    const uint8_t kProductId = 0x5F;

    // Facteurs de conversion
    const float kMeterToInch = 39.37f;
    const float kInchToMeter = 1.0f / kMeterToInch;
    const float kRadianToDegree = 180.0f / 3.14159f;
    const float kDegreeToRadian = 3.14159f / 180.0f;

    // Facteurs de conversion pour les registres de position linéaire
    const float kMeterToInt16 = 32768.0f / 10.0f;
    const float kInt16ToMeter = 1.0f / kMeterToInt16;

    // Facteurs de conversion pour les registres de vitesse linéaire
    const float kMpsToInt16 = 32768.0f / 5.0f;
    const float kInt16ToMps = 1.0f / kMpsToInt16;

    // Facteurs de conversion pour les registres d'accélération linéaire
    const float kMpssToInt16 = 32768.0f / (16.0f * 9.80665f);
    const float kInt16ToMpss = 1.0f / kMpssToInt16;

    // Facteurs de conversion pour les registres de position angulaire
    const float kRadToInt16 = 32768.0f / 3.14159f;
    const float kInt16ToRad = 1.0f / kRadToInt16;

    // Facteurs de conversion pour les registres de vitesse angulaire
    const float kRpsToInt16 = 32768.0f / (2000.0f * kDegreeToRadian);
    const float kInt16ToRps = 1.0f / kRpsToInt16;

    // Facteurs de conversion pour les registres d'accélération angulaire
    const float kRpssToInt16 = 32768.0f / (3.14159f * 1000.0f);
    const float kInt16ToRpss = 1.0f / kRpssToInt16;

    // Valeurs scalaires minimales et maximales
    const float kMinScalar = 0.872f;
    const float kMaxScalar = 1.127f;
};


#endif /* INC_DEVICES_QWIICOTOS_H_ */
