#ifndef STM_MAIN_BOARD_COLORSENSOR_H
#define STM_MAIN_BOARD_COLORSENSOR_H

#include "main.h"
#include "i2c.h"

inline const char* to_c_str(const com_types::TeamColor color)
{
    switch (color)
    {
    case com_types::TeamColor::YELLOW:  return "YELLOW";
    case com_types::TeamColor::BLUE:    return "BLUE";
    case com_types::TeamColor::UNKNOWN: default : return "UNKNOWN";
    }
}

class ColorSensorTCS34725
{
public:
    bool init()
    {
        // Check presence of color sensor TCS34725
        if (readReg(0x12) != ID) return false;

        // Power on + ADC
        writeReg(0x00, 0x01); // PON
        HAL_Delay(3);
        writeReg(0x00, 0x01 | 0x02); // PON + AEN

        // Settings : integration (50ms), gain(4x)
        writeReg(0x01, 0xEB); // ATIME: 50ms
        writeReg(0x0F, 0x01); // AGAIN: 4x
        return true;
    }

    com_types::TeamColor detectColor() const
    {
        uint8_t data[8];
        if (HAL_I2C_Mem_Read(_hi2c, ADDR, 0x14 | 0x80, 1, data, 8, 100) != HAL_OK)
            return com_types::TeamColor::UNKNOWN;

        // Convert bytes to 16bits values
        const uint16_t c = (data[1] << 8) | data[0];
        const uint16_t r = (data[3] << 8) | data[2];
        const uint16_t g = (data[5] << 8) | data[4];
        const uint16_t b = (data[7] << 8) | data[6];

        if (c < 50) return com_types::TeamColor::UNKNOWN; // Proximity threshold

        // Compute ratios to be independent of luminosity
        const float r_ratio = static_cast<float>(r) / c;
        const float b_ratio = static_cast<float>(b) / c;
        const float g_ratio = static_cast<float>(g) / c;

        // Yellow: lots of red + green. less blue
        if (r_ratio > 0.40f && g_ratio > 0.40f && b_ratio < 0.25f)
            return com_types::TeamColor::YELLOW;

        // Blue
        if (b_ratio > 0.40f)
            return com_types::TeamColor::BLUE;

        return com_types::TeamColor::UNKNOWN;
    }

    static com_types::TeamColor inverseColors(const com_types::TeamColor color)
    {
        if (color == com_types::TeamColor::BLUE)
            return com_types::TeamColor::YELLOW;
        if (color == com_types::TeamColor::YELLOW)
            return com_types::TeamColor::BLUE;
        return com_types::TeamColor::UNKNOWN;
    }

private:
    static constexpr uint8_t ADDR = 0x29 << 1;  // Default I2C address
    static constexpr uint8_t ID   = 0x44;       // Default ID // TODO or maybe 0x4D --> to check
    I2C_HandleTypeDef* _hi2c = &hi2c3;

    void writeReg(uint8_t reg, uint8_t val)
    {
        uint8_t cmd = 0x80 | reg; // Protocol bit
        HAL_I2C_Mem_Write(_hi2c, ADDR, cmd, 1, &val, 1, 10);
    }
    uint8_t readReg(uint8_t reg) const
    {
        uint8_t val, cmd = 0x80 | reg; // Protocol bit
        HAL_I2C_Mem_Read(_hi2c, ADDR, cmd, 1, &val, 1, 10);
        return val;
    }
};


#endif //STM_MAIN_BOARD_COLORSENSOR_H