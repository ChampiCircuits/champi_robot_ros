#ifndef STM_MAIN_BOARD_COLORSENSOR_H
#define STM_MAIN_BOARD_COLORSENSOR_H

class ColorSensor
{
public:
    com_types::TeamColor getColor(bool do_inverse);
private:
    static com_types::TeamColor _inverseColors(com_types::TeamColor color);
};

inline com_types::TeamColor ColorSensor::getColor(bool do_inverse)
{
    com_types::TeamColor color = com_types::TeamColor::BLUE;
    // TODO IMPLEMENT

    if (do_inverse)
        return _inverseColors(color);
    return color;
}

inline com_types::TeamColor ColorSensor::_inverseColors( com_types::TeamColor color)
{

    if (color == com_types::TeamColor::BLUE)
        return com_types::TeamColor::YELLOW;
    if (color == com_types::TeamColor::YELLOW)
        return com_types::TeamColor::BLUE;
    return com_types::TeamColor::UNKNOW;
}

#endif //STM_MAIN_BOARD_COLORSENSOR_H
